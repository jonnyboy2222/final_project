import socket
import threading
import cv2
import time
import serial
import speech_recognition as sr
import tempfile
import os

# ====== 전역 제어 플래그 ======
stop_vision_event = threading.Event()
stop_chat_event = threading.Event()


class ARCSDataService:
    def __init__(self):
        # Vision 서버 (UDP)
        self.udp_ip = '192.168.0.252'
        self.udp_port = 54321

        self.camera = 0
        # self.camera == 0 if "LD" self.camera == 1 if "FW"

        # LLM 서버 (TCP)
        self.tcp_ip = '192.168.0.33'
        self.tcp_llm_port = 54322

        self.running = True
        self.is_active = True  # 음성 인식 활성화 상태

    # 영상 -> vision 서버(UDP):  영상 전송 스레드
    def video_to_vision_server(self):

        cap = cv2.VideoCapture(self.camera)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)

        udp_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        while self.running and cap.isOpened():

            if stop_vision_event.is_set():
                print("[Vision] 중단 명령 수신 → 전송 종료")
                break

            ret, frame = cap.read()

            if not ret:
                break

            frame = cv2.resize(frame, (320, 320))

            ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])

            if not ret:
                continue

            udp_client.sendto(buffer.tobytes(), (self.udp_ip, self.udp_port))

        cap.release()
        udp_client.close()
        cv2.destroyAllWindows()

    def convert_camera(self, cmd):
        if cmd == "FW":
            self.camera = 1
        elif cmd == "LD":
            self.camera = 0

    # 음성 인식 및 텍스트 전송
    def voice_to_llm_server(self):
        recognizer = sr.Recognizer()
        mic = sr.Microphone(device_index=9)  # 적절히 조정

        print("[Voice] Wake Word '아크야' 대기 중...")

        while self.running:
            if stop_chat_event.is_set():
                print("[Chat] 중단 명령 수신 → 음성 전송 종료")
                break

            try:
                with mic as source:
                    recognizer.adjust_for_ambient_noise(source, duration=0.2)
                    print("[Voice] 음성 듣는 중...")
                    audio = recognizer.listen(source, timeout=7, phrase_time_limit=5)

                try:
                    text = recognizer.recognize_google(audio, language="ko-KR")

                except (sr.UnknownValueError, sr.RequestError, ValueError) as e:
                    print(f"[STT 인식 실패] {e}")
                    continue

                print(f"[Voice] 인식됨: {text}")

                #상태 따라 처리
                if not self.is_active:
                    if "아크야" in text:
                        print("[Voice] Wake 상태 진입")
                        self.is_active = True

                else:
                    if "아크야 이제 들어가" in text:
                        print("[Voice] Sleep 상태 전환")
                        self.is_active = False
                    else:
                        self.send_text_to_llm_server(text)

            except sr.WaitTimeoutError:
                continue
            except Exception as e:
                print(f"[Voice ERROR] {e}")
                time.sleep(1)

    def send_text_to_llm_server(self, text):
        try:
            message = text.encode("utf-8")
            header = b'\x00'
            length = len(message).to_bytes(4, byteorder='big')

            packet = header + length + message

            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.connect((self.tcp_ip, self.tcp_llm_port))
                sock.sendall(packet)
                print(f"[Voice] 텍스트 전송 완료: {text}")

        except Exception as e:
            print(f"[TCP ERROR] {e}")

    # ===== 시리얼 수신기 =====
    def serial_listener(self):
        SERIAL_PORT = '/dev/ttyUSB0'  # 포트 확인 필요
        BAUDRATE = 115200
        try:
            with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as ser:
                print(f"[Serial] Listening on {SERIAL_PORT}...")
                while self.running:
                    data = ser.read(3)

                    if len(data) < 3:
                        continue

                    header = data[0:1]
                    cmd = data[1:].decode('utf-8', errors='ignore').strip()
                    print(f"[Serial] 수신 명령어: {cmd}")
                    self.handle_serial_command(cmd)

        except serial.SerialException as e:
            print(f"[Serial] 시리얼 오류: {e}")

    def handle_serial_command(self, cmd: str):
        if cmd in ["FW", "LD"]:
            self.convert_camera(cmd)

        elif cmd in ["PS", "RT"]:
            print("[Serial] 전송 중단 명령 수신 → Vision/Chat 중지")
            stop_vision_event.set()
            stop_chat_event.set()

        elif cmd == "RS":
            print("[Serial] resume 명령 수신 → Vision/Chat 재개")
            stop_vision_event.clear()
            stop_chat_event.clear()

        else:
            print(f"[Serial] 알 수 없는 명령: {cmd}")

    # ===== 스레드 실행 =====
    def start_all(self):
        threading.Thread(target=self.video_to_vision_server, daemon=True).start()
        threading.Thread(target=self.voice_to_llm_server, daemon=True).start()
        threading.Thread(target=self.serial_listener, daemon=True).start()

    # ===== 메인 루프 =====
    def main_loop(self):
        try:
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            self.running = False
            print("Shutting down...")

if __name__ == "__main__":
    controller = ARCSDataService()
    controller.start_all()
    controller.main_loop()