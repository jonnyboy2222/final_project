import socket
import struct
import threading
import json
import numpy as np
import cv2
import queue
import time

import speech_recognition as sr
import tempfile
import os

import platform
from gtts import gTTS
import subprocess

# user pc, admin pc 데이터 전송 (user pc는 queue로, admin쪽은 tcp로)
# user pc에서 tcp로 데이터 수신 (gui에서 queue에 데이터 넣고 빼올예정)

class ARCSManager:
    def __init__(self):
        self.udp_ip = '192.168.2.30'
        self.udp_port = 54321

        self.tcp_ip = '0.0.0.0'

        # Vision용 포트
        self.vision_tcp_port = 12345
        # LLM용 포트
        self.llm_tcp_port = 12346

        self.admin_ip = '192.168.2.40'  # 예시 Admin PC IP
        self.admin_port = 23456         # 예시 Admin TCP 포트

        self.running = True
        self.admin_conn = None
        self.voice_data_queue = queue.Queue()
        self.gui_data_queue = queue.Queue()

    # 1. 영상 → Vision 서버 (UDP)
    def video_to_vision_server(self):
        cap = cv2.VideoCapture(0)
        udp_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        while self.running and cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if not ret:
                continue

            udp_client.sendto(buffer.tobytes(), (self.udp_ip, self.udp_port))
            cv2.imshow("Sending...", frame)
            if cv2.waitKey(1) == 27:  # ESC
                self.running = False
                break

        cap.release()
        udp_client.close()
        cv2.destroyAllWindows()

    # 2. 음성 → LLM 서버 (TCP 전송 예시용)
    def voice_to_llm_server(self):
        recognizer = sr.Recognizer()
        mic = sr.Microphone(device_index=4)  # 인덱스는 상황에 맞게 수정
        self.is_active = False

        print("[Voice] Wake Word '아크야' 대기 중...")

        while self.running:
            try:
                with mic as source:
                    recognizer.adjust_for_ambient_noise(source, duration=0.5)
                    print("[Voice] 음성 듣는 중...")
                    audio = recognizer.listen(source, timeout=7, phrase_time_limit=5)

                # 텍스트 변환 시도
                with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as f:
                    f.write(audio.get_wav_data())
                    wav_path = f.name

                with sr.AudioFile(wav_path) as source:
                    audio_data = recognizer.record(source)
                    try:
                        text = recognizer.recognize_google(audio_data, language="ko-KR")
                    except (sr.UnknownValueError, sr.RequestError, ValueError) as e:
                        print(f"[STT 인식 실패] {e}")
                        text = ""

                print(f"[Voice] 인식됨: {text}")
                os.remove(wav_path)

                # 상태 판단
                if not self.is_active:
                    if "헤이 아크" in text:
                        print("[Voice] Wake 상태 진입")
                        self.is_active = True
                else:
                    if "바이 아크" in text:
                        print("[Voice] Sleep 상태 전환")
                        self.is_active = False
                    else:
                        self.send_audio_to_llm_server(audio)

            except sr.WaitTimeoutError:
                continue
            except Exception as e:
                print(f"[Voice ERROR] {e}")
                time.sleep(1)

    def send_audio_to_llm_server(self, audio_data):
        try:
            with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as f:
                f.write(audio_data.get_wav_data())
                wav_path = f.name

            with open(wav_path, "rb") as f:
                wav_bytes = f.read()

            os.remove(wav_path)

            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.connect((self.udp_ip, self.llm_tcp_port))
                sock.sendall(len(wav_bytes).to_bytes(4, byteorder='big'))
                sock.sendall(wav_bytes)
                print(f"[Voice] 음성 전송 완료 ({len(wav_bytes)} bytes)")

        except Exception as e:
            print(f"[Voice TCP ERROR] {e}")

    # 3. 양쪽 서버에서 TCP 데이터 수신
    def start_vision_tcp_server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.bind((self.tcp_ip, self.vision_tcp_port))
            server.listen()
            print(f"[Vision TCP] Listening on {self.tcp_ip}:{self.vision_tcp_port}")

            while self.running:
                conn, addr = server.accept()
                threading.Thread(target=self.vision_tcp_receiver, args=(conn, addr), daemon=True).start()

    def vision_tcp_receiver(self, conn, addr):
        print(f"[Vision] Connected from {addr}")
        try:
            while self.running:
                data = conn.recv(4096)
                if not data:
                    break
                decoded = data.decode()
                print(f"[Vision] Received:", decoded)

                # 예: JSON 데이터 처리
                try:
                    parsed = json.loads(decoded)
                    # parsed = {'bbox': [...], 'track_id': ...}
                    self.gui_data_queue.put(parsed)
                except Exception as e:
                    print("[Vision] JSON Parse Error:", e)

        finally:
            conn.close()

    def start_llm_tcp_server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.bind((self.tcp_ip, self.llm_tcp_port))
            server.listen()
            print(f"[LLM TCP] Listening on {self.tcp_ip}:{self.llm_tcp_port}")

            while self.running:
                conn, addr = server.accept()
                threading.Thread(target=self.llm_tcp_receiver, args=(conn, addr), daemon=True).start()

    def speak_text(self, text):
        try:
            print(f"[TTS] 읽기: {text}")
            tts = gTTS(text=text, lang='ko')
            with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as fp:
                tts.save(fp.name)
                path = fp.name

            if platform.system() == "Windows":
                os.system(f'start /wait {path}')
            elif platform.system() == "Darwin":  # macOS
                os.system(f'afplay {path}')
            else:  # Linux
                os.system(f'mpg123 {path}')

            os.remove(path)
        except Exception as e:
            print(f"[TTS ERROR] {e}")

    def llm_tcp_receiver(self, conn, addr):
        print(f"[LLM] Connected from {addr}")
        try:
            data = b""
            while True:
                packet = conn.recv(1024)
                if not packet:
                    break
                data += packet

            if data:
                text = data.decode("utf-8").strip()
                print(f"[LLM] Message: {text}")
                self.gui_data_queue.put({"llm_response": text})

                # gTTS로 음성 출력
                self.speak_text(text)

        except Exception as e:
            print(f"[LLM Receiver ERROR] {e}")
        finally:
            conn.close()


    # 4. Admin PC로 데이터 전송
    def data_to_admin_pc(self):
        while self.running:
            try:
                if self.admin_conn is None:
                    self.admin_conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.admin_conn.connect((self.admin_ip, self.admin_port))
                    print("[Admin] Connected to Admin PC")

                if not self.gui_data_queue.empty():
                    msg = self.gui_data_queue.get()
                    self.admin_conn.sendall(json.dumps(msg).encode("utf-8"))

            except Exception as e:
                print(f"[Admin ERROR] {e}")
                if self.admin_conn:
                    self.admin_conn.close()
                    self.admin_conn = None
                time.sleep(1)

    # 스레드 시작
    def start_all(self):
        threading.Thread(target=self.video_to_vision_server, daemon=True).start()
        threading.Thread(target=self.voice_to_llm_server, daemon=True).start()
        threading.Thread(target=self.start_vision_tcp_server, daemon=True).start()
        threading.Thread(target=self.start_llm_tcp_server, daemon=True).start()
        threading.Thread(target=self.data_to_admin_pc, daemon=True).start()

        # 메인 루프 유지
        try:
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            self.running = False
            print("Shutting down...")

if __name__ == "__main__":
    manager = ARCSManager()
    manager.start_all()
