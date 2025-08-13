import sys
import cv2
import datetime
import socket
import json, yaml
import re
import struct
import numpy as np
import time
from gtts import gTTS
import tempfile
import os, threading, subprocess
import platform
from collections import deque
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLabel, QPushButton, QTableWidgetItem, QProgressBar,
    QGraphicsView, QGraphicsScene, QGraphicsPixmapItem, QGraphicsEllipseItem, QGraphicsLineItem
    )
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, Qt, QPointF, QObject, QMetaObject
from PyQt5.uic import loadUi
from PyQt5.QtGui import QImage, QPixmap, QPen, QBrush, QTransform
import queue

class TcpCommunicator(QObject):
    #메인에서 사용하기 위해서
    received_OO = pyqtSignal(dict)   
    received_AR = pyqtSignal(dict)
    received_XX = pyqtSignal(dict)

    def __init__(self,send_ip='192.168.0.145', receive_ip='0.0.0.0', send_port=12347, receive_port=23457):
        super().__init__()
        self.send_ip = send_ip
        self.send_port = send_port

        self.receive_ip = receive_ip
        self.receive_port = receive_port

        self._running_tcp = True

# =====================================================================================================
#init 송신 스레드/큐 추가 
# =====================================================================================================
        self.send_queue = queue.Queue()
        self.sender_socket = None
        self.sender_thread = threading.Thread(target=self._send_loop, daemon=True) #daemon=True :보조 작업용 스레드임을 표시
        self.sender_thread.start() #sender_thread에서 self._send_loop 바로 실행되기 시작

# =====================================================================================================
#init 수신 관련
# =====================================================================================================
        self.receiver = None
        self.receiver_conn = None

        #수신용 서버 소켓 (mu_* 수신용)
        try:
            self.receiver = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            #소켓 재사용 설정(소켓 닫은 직후에도 포트 재사용 가능하게)
            self.receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
            #지정된 IP/PORT에 바인딩 (서버로 열 준비)
            self.receiver.bind((receive_ip, receive_port))
            #연결 대기 상태로 진입 (클라이언트의 connect를 기다림)
            self.receiver.listen(1)
            #바인딩 성공 후 로그 출력
            print(f"[TCP]수신서버소켓 생성완료 → {receive_ip}:{receive_port}")
        except Exception as e:
            print(f"[TCP] 수신서버소켓 설정 실패: {e}")
            self.receiver = None
    
        #수신 연결 대기 (별도 스레드로)
        self.accept_thread = threading.Thread(target=self._accept_and_receive_forever)
        self.accept_thread.start()


# ========================================================================================
# 송신
# ========================================================================================
    #보낼 tcp를 큐에 담기
    def send_tcp_message_um(self, command, payload_to_send):
        #소켓을 직접 열지 않고, 큐에만 쌓는다!
        self.send_queue.put((command, payload_to_send))

    #송신 전용 스레드. 큐에 쌓인 메시지 → sender_socket으로 전송.
    def _send_loop(self):
        while self._running_tcp:
            try:
                # 송신 소켓 연결 (없으면 새로 연결)
                if self.sender_socket is None:
                    print(f"[송신] 송신 소켓이 없습니다. {self.send_ip}:{self.send_port} 에 연결을 시도합니다...")
                    self.sender_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.sender_socket.connect((self.send_ip, self.send_port))
                    print(f"[송신] {self.send_ip}:{self.send_port} 에 성공적으로 연결되었습니다.")

                 # 메시지가 있을 때까지 대기 (0.5초마다 체크)
                try:
                    command, payload_to_send = self.send_queue.get(timeout=0.5)
                except queue.Empty:
                    continue  # 메시지 없으면 루프 계속

                header = b'\x00' #1byte #0x00
                command_bytes = command.encode("utf-8")  #2bytes #ex) command = "KG" → command_bytes = b'KG' 
                #payload가 존재할 때만
                if payload_to_send is not None:
                    payload_json = json.dumps(payload_to_send).encode("utf-8") #*bytes 
                                                                            #payload_to_send = {"loadcell": 12345}
                                                                            #json.dumps(...) → '{"loadcell": 12345}'
                                                                            #.encode("utf-8") → b'{\"loadcell\": 12345}'
                    length = struct.pack('>I',len(payload_json)) #json 길이
                    message = header + length + command_bytes + payload_json

                else: #payload_to_send가 없을때
                    length = struct.pack('>I', 0)
                    message = header + length + command_bytes
                self.sender_socket.sendall(message)



            except (ConnectionResetError, BrokenPipeError) as e:
                print(f"[송신][ERROR] 송신 소켓 연결이 끊어졌습니다: {e!r}")
                print("[송신] 1초 후에 재연결을 시도합니다...")
                if self.sender_socket:
                    self.sender_socket.close()
                self.sender_socket = None
                time.sleep(1)
            except Exception as e:
                print(f"[송신][EXCEPTION] 예상치 못한 오류가 발생했습니다: {e!r}")
                print("[송신] 이 오류는 재연결을 시도하지 않고, 루프를 계속 진행합니다.")
   
# =======================================================================================================
# 수신
# =======================================================================================================
    #수신 연결 & 메시지 수신을 영원히 반복(running=True면) [자동재연결]
    def _accept_and_receive_forever(self):
        while self._running_tcp:
            try:
                # print("[TCP] 클라이언트 연결 대기 중...")
                #클라이언트의 연결 수락 (block 상태로 기다리는 상태임)
                self.receiver_conn, addr = self.receiver.accept() #self.receiver_conn : 실제 통신용 소켓, addr: 연결된 클라이언트의 주소 정보
                #연결 성공 시 연결된 클라이언트 주소 출력
                print(f"[TCP] 수신 연결 성공! receiver_conn생성완료 : 연결된 클라이언트: {addr}")

                while self.receiver_conn:
                    # print("수신 쓰레딩 러닝 중")
                    try:
                        #header 수신
                        header = self.receiver_conn.recv(1) #수신 버퍼의 가장 앞1바이트 읽어오기
                        if not header:
                            raise ConnectionResetError("header 없음, 연결 종료")
                        if header == b'\x00':
                            pass
                        else:
                            print(f"{header} received: 헤더 이상하게 생겼는데?")

                        #lenth_bytes 수신
                        length_bytes = self.receiver_conn.recv(4) # 그 다음 4바이트를 읽어오기 # '\x00\x00\x00\x14'
                        if len(length_bytes) < 4:
                            raise ConnectionResetError("length_bytes 4바이트 미만, 연결 종료")
                        (length,) = struct.unpack('>I', length_bytes) #'\x00\x00\x00\x14'를 int로 바꿔준다

                        #command 수신
                        command_bytes = self.receiver_conn.recv(2) #ex)  b'AR'
                        command = command_bytes.decode("utf-8") #ex) 'AR'
                        print("[TCP 수신 받으면 띄움]: " + command)

                        #json수신 및 해독
                        data = b''
                        while len(data) < length:
                            packet = self.receiver_conn.recv(length - len(data))
                            if not packet:
                                raise ConnectionResetError("payload 없음, 연결 종료")
                            data += packet
                        payload_dict = json.loads(data.decode("utf-8")) #gui에서 직접 사용할 수 있는 수신데이터 형식
                        print(f" [json 해독 완료]: {payload_dict}")

                        #mu_2 수신
                        if command == "AR":
                            print("AR 들어감")
                            self.received_AR.emit(payload_dict)
                        #mu_3 처리(llm목적지 처리해야 할 때)
                        elif command == "XX":
                            print("XX 들어감")
                            self.received_XX.emit(payload_dict)
                        #mu_1 처리(일반 목적지 처리해야 할 때)
                        elif command == "OO":
                            # print("OO 들어감")
                            self.received_OO.emit(payload_dict)
                    
                    except Exception as e:
                        print(f"[TCP 수신 오류 - 내부]: {e}")
                        if self.receiver_conn:
                            try:
                                self.receiver_conn.shutdown(socket.SHUT_RDWR)
                                self.receiver_conn.close()
                            except:
                                pass
                            self.receiver_conn = None
                        break  # 바깥 while로 빠져나가 재연결 시도
            except Exception as e:
                print(f"[TCP 수신 오류 - accept]: {e!r}")
                time.sleep(1) #1초 대기
            time.sleep(1) #매 루프 끝마다 항상 쉬어 가며 전체 루프 속도를 제한하는 용도
           
# ===============================================================================================
# 닫아
# ===============================================================================================
    def close(self):
        try:
            self._running_tcp = False

             # 송신 소켓 종료
            if self.sender_socket:
                try:
                    #양방향 모두 종료 신호(FIN)를 상대에 보내고, 이후 모든 입출력이 차단
                    self.sender_socket.shutdown(socket.SHUT_RDWR) #rd(수신),wr(송신) 하지 않겠다 선언(fin패킷 전송)
                    #소켓 해제(fd해제)
                    self.sender_socket.close()
                    print("[TCP 닫는중] sender_socket(송신용) 닫힘")
                except Exception:
                    pass

            if self.receiver_conn:
                try:
                    self.receiver_conn.shutdown(socket.SHUT_RDWR)
                    self.receiver_conn.close()
                    self.receiver_conn = None
                    print("[TCP 닫는중] receiver_conn(수신연결소켓) 닫힘")
                except:
                    print("[TCP 닫는중] receiver_conn(수신연결소켓) 안 닫힘")

            if self.receiver:
                try:
                    self.receiver.shutdown(socket.SHUT_RDWR)
                    self.receiver.close()
                    self.receiver = None
                    print("[TCP 닫는중]receiver(수신 서버 소켓) 닫힘")
                except:
                    print("[TCP 닫는중]receiver(수신 서버 소켓) 안 닫힘")

            if hasattr(self, 'accept_thread'):
                try:
                    self.accept_thread.join(timeout=2.0) #최대 2초까지 쓰레드 꺼지는 것을 기다림
                    print("[TCP닫는중] 수신 스레드 종료 완료")
                except:
                    print("[TCP닫는중] 수신 스레드 종료 미완료")

        except Exception as e:
            print(f"[TCP 소켓 닫기 오류]: {e}")

# =========================================================================================================
# qr카메라 쓰레드
# =========================================================================================================
# Camera thread with QR detection
class CameraWorker(QObject):
    # 카메라 프레임을 QImage로 전달하기 위한 시그널
    frame_received = pyqtSignal(QImage)
    # QR 코드가 인식되면 문자열을 전달하기 위한 시그널
    qr_detected = pyqtSignal(str)

    def __init__(self, index=0):
        super().__init__()  
        self.index = index                       
        self._run_qr_camera = False
        self.qr_thread = None                        
        self.detector = cv2.QRCodeDetector() # QR 코드 디텍터 초기화

    def start(self):
        if self._run_qr_camera:
            return
        self._run_qr_camera = True
        self.qr_thread = threading.Thread(target=self._camera_loop, daemon = True)
        self.qr_thread.start()

    def _camera_loop(self):
        cap = cv2.VideoCapture(self.index)                                   
        while self._run_qr_camera:                          
            ret, frame = cap.read()                  
            if not ret:                              
                continue
            # --- QR 코드 인식 파트 ---
            data, bbox, _ = self.detector.detectAndDecode(frame)  #  프레임에서 QR 디코딩
                                                       
            if data:                                 
                self.qr_detected.emit(data)         # 인식된 str을 UI로 전송
                self._run_qr_camera = False                 

            # --- 화면 표시용 이미지 변환 파트 ---
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                                                    
            h, w, ch = rgb.shape                     # 이미지 높이, 너비, 채널 수 추출
            # 원시 RGB 픽셀 데이터를 QImage로 변환
            qimg = QImage(rgb.data, w, h, ch*w, QImage.Format_RGB888).copy()
            self.frame_received.emit(qimg)            #  QImage를 UI로 전송

        cap.release()                               

    def stop(self):
        self._run_qr_camera = False
        if self.qr_thread is not None:
            self.qr_thread.join()
            self.qr_thread = None

# ========================================================================================================
# 메인이다
# ========================================================================================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        loadUi("/home/dong/project_local/final_project/user_gui/client.ui", self)
         # TCP 통신 객체 생성
        self.tcp = TcpCommunicator()

        #메인에서 handle함수 다루기 위해.
        self.tcp.received_OO.connect(self.handle_mu1)
        self.tcp.received_AR.connect(self.handle_mu2)
        self.tcp.received_XX.connect(self.handle_mu3)

        # ─── TTS 전용 워커 설정 ───
        self._tts_queue = deque()             # 재생 대기열
        self.tts_playing = False              # 재생 중 플래그
        self.tts_process = None               # 현재 mpg123 프로세스
        self._tts_worker_running = True       # 워커 루프 제어 플래그
        self._tts_worker = threading.Thread(target=self._tts_worker_loop, daemon=True)
        self._tts_worker.start()
        self.before_state_f = 0
        self.before_state_g = 0

        #목적지 맵핑
        self.place_coords ={             
            'gate1': (5.0, 13.1, 0.0),
            'gate2': (2.5, 10.0, 0.0),
            'gate3': (5.7, 5.7, 0.0),
            'toilet_w': (0.0, 0.0, 0.0),
            'toilet_e': (5.7, 8.7, 0.0), 
            'shop_S': (1.0, 0.0, 0.0),
            'shop_L': (2.5, 0.0, 0.0),
            'shop_H': (4.0, 0.0, 0.0),
            'restaurant_K': (2.5, 7.3, 0.0),
            'restaurant_C': (5.5, 0.0, 0.0),
            'convenience_store': (4.0, 11.4, 0.0),
            'info_desk': (4.0, 8.7, 0.0),
            'cafe': (4.0, 5.7, 0.0)
        }

       
        self.destinations = []  #a*에서 쓰일 목적지
        self.selected_keys = [] #page_selected_place에서 버튼 색 조절하려고 사용
        self.arrived_place = None #도착한 장소
        self.arrived_key = None #도착한 장소 이름
        self.edit = False   #목적지 수정

        # decision 페이지 용 타이머 (싱글샷)
        self.decision_timer = QTimer(self)
        self.decision_timer.setSingleShot(True)
        self.decision_timer.timeout.connect(self._on_decision_timeout)

        #가이드 중 일시정지 사용 위해서 
        self.is_paused_g = False
        #팔로잉 중 일시정지 사용 위해서
        self.is_paused_f = False
        
        # camera
        self.camera = CameraWorker(0)  #카메라 쓰레드 생성
        self.scanned_data = []
        self.user_info = {}
        self.camera.frame_received.connect(self.on_camera_frame)
        self.camera.qr_detected.connect(self._on_qr_detected)

        self.scene = QGraphicsScene()
        with open("/home/dong/project_local/final_project/user_gui/final_map.yaml", 'r') as f:
            meta = yaml.safe_load(f)
            self.resolution = meta['resolution']  #1픽셀당 실제 몇 미터(또는 cm)에 해당하는지
            self.origin = meta['origin'][:2]  # 지도 이미지의 (0,0) 픽셀이 실제 세계의 어떤 지점 (x, y)에 대응하는지를 정의하는 부분
            # print(f"[MapMeta] resolution = {self.resolution}")
            # print(f"[MapMeta] origin     = {self.origin}")
        self.map_image = QPixmap("/home/dong/project_local/final_project/user_gui/final_map.pgm")
        transform = QTransform().rotate(90)
        self.map_image = self.map_image.transformed(transform)
        self.map_item = QGraphicsPixmapItem(self.map_image)
        self.scene.addItem(self.map_item)
        self.arcs_item = QGraphicsEllipseItem(-3, -3, 6, 6) #중심이 (0,0)인 지름 10px짜리 원 객체 생성
        self.arcs_item.setBrush(QBrush(Qt.red)) #원 내부 색 빨강으로
        self.scene.addItem(self.arcs_item) #scene에다가 빨간 원 추가. 아직 시각화 안 해서 보이지는 않음
        #경로 선 저장용
        self.path_items = []
        self.path = []   #경로 
        self.position = None

        # page mapping
        self.pages = {
            'main': self.page_main,
            'scan': self.page_scan,
            'check_info': self.page_check_info,
            'loading': self.page_loading,
            'following': self.page_following,
            'select_place': self.page_select_place,
            'check_goal': self.page_check_goal,
            'guiding': self.page_guiding,
            'decision': self.page_decision_after_arrived,
            'waiting': self.page_waiting,
            'user_valid': self.page_user_valid,
            'bye': self.page_bye,
            'charging': self.page_charging,
        }

        # 페이지별 연결
        self._connect_page_main()
        self._connect_page_scan()
        self._connect_page_check_info()
        self._connect_page_loading()
        self._connect_page_following()
        self._connect_page_select_place()
        self._connect_page_check_goal()
        self._connect_page_guiding()
        self._connect_page_decision()
        self._connect_page_waiting()
        self._connect_page_user_valid()
        self._connect_page_bye()
        self._connect_page_charging()

        # 초기 페이지
        self.current_page = None
        self.before_page = None
        self.show_page('main')

        #실시간 상단바 시계
        self.topbar_time_timer = QTimer(self)
        self.topbar_time_timer.timeout.connect(self.update_datetime)
        self.topbar_time_timer.start(1000)

        # 홈 버튼 누를 경우
        for btn in self.findChildren(QPushButton):
            if btn.objectName().startswith("btn_home"):
                btn.clicked.connect(self.on_home_clicked)

    #상단바 날짜 및 시간 업데이트 함수
    def update_datetime(self):
        now = datetime.datetime.now()
        weekdays = ['월','화','수','목','금','토','일']
        wd = weekdays[now.weekday()]
        txt = now.strftime(f"%Y.%m.%d ({wd}) %H:%M")
        for lbl in self.findChildren(QLabel):
            if lbl.objectName().startswith("label_datetime"):
                lbl.setText(txt)

    #홈 버튼 누르면 하는 일
    def on_home_clicked(self):
        self.reset_all()
        #메인으로
        payload_to_send = {
            "robot_id": 1,
            "user_info": None,
            "des_coor": None
        }
        self.tcp.send_tcp_message_um("PS", payload_to_send)
        print("[TCP] PS 전송 완료")
        self.show_page('main')

    #초기화
    def reset_all(self):
        self.before_state_f = 0
        self.before_state_g = 0
        self.tts_playing = False
        self.tts_process = None 
        #스캔 관련
        self.scanned_data = []
        self.user_info = {}
        #선택한 기능 관련
        self.selected_function = None
        #인증번호 관련
        self.correct_password = ""
        self.pushed_valid_btn = ""
        #목적지 관련
        for item in list(self.scene.items()):
            if isinstance(item, QGraphicsLineItem):
                self.scene.removeItem(item)
        self.path = []   #경로
        self.path_items = [] #경로에 나타내는 직선 저장용
        self.destinations = []  #목적지
        self.selected_keys = [] #선택한 목적지
        self.arrived_place = None
        self.arrived_key = None
        for btn in self.findChildren(QPushButton):
            if btn.objectName().startswith("btn_place_"):
                btn.setStyleSheet("")
        #일시정지 관련
        self.is_paused_g = False
        self.is_paused_f = False
        #목적지 수정
        self.edit = False
        #버튼 텍스트 변경
        self.btn_pause5.setText("일시정지")
        self.btn_pause8.setText("일시정지")

    # 페이지 전환 할 때 쓰이는 함수
    def show_page(self, page_key):
        # 페이지 전환 시, 이전에 재생 중이던 TTS 중단
        self._stop_tts()
        # 이전 페이지 종료(_exit_함수의 기능 수행)
        self.before_page = self.current_page
        if self.before_page:
            fn = getattr(self, f"_exit_{self.before_page}", None)
            if fn: fn()
        # 화면 전환
        widget = self.pages[page_key]
        self.stackedWidget.setCurrentWidget(widget)
        self.current_page = page_key
        # 새 페이지 진입 로직(_enter_함수의 기능 수행)
        fn = getattr(self, f"_enter_{page_key}", None)
        if fn: fn()

# ==============================================================================
# 말 좀 해볼게요
# ==============================================================================
    # TTS 실행 요청 (중복 실행 방지)
    def play_tts_async(self, text):
        if not text:
            return
        
        #큐에 마지막으로 들어간 메시지와 같으면 무시
        if self._tts_queue and self._tts_queue[-1] == text:
            print("[TTS] 중복 메시지, 큐에 추가하지 않음")
            return
        
        #큐에 메세지 추가
        self._tts_queue.append(text)
  
    def _tts_worker_loop(self):
        while self._tts_worker_running:
            if not self._tts_queue or self.tts_playing:
                time.sleep(0.1)
                continue

            msg = self._tts_queue.popleft()
            self.tts_playing = True
            path = None

            try:
                # 1) MP3 파일 생성
                tts = gTTS(text=msg, lang='ko')
                with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as fp:
                    tts.save(fp.name)
                    path = fp.name
                
                # 2) 외부 프로세스로 재생
                proc = subprocess.Popen(["mpg123", "-q", path],
                                        stdout=subprocess.DEVNULL,
                                        stderr=subprocess.DEVNULL)  #실제 음성이 나오는 부분
                self.tts_process = proc
                proc.wait()  # 재생이 끝날 때까지 블록
            except Exception as e:
                print(f"[TTS ERROR] {e}")
            finally:
                # 3) 정리
                if path:
                    try: os.remove(path)
                    except: pass
                self.tts_playing = False
                self.tts_process = None

    def _stop_tts(self):
        self._tts_queue.clear()
        #재생 중인 TTS(mp3) 프로세스를 강제 종료하고 상태 초기화
        if self.tts_process:
            try: self.tts_process.terminate()
            except: pass
            self.tts_process = None
        self.tts_playing = False

# =============================================================================
    #tcp 수신 했을 때 처리
# =============================================================================
    #tcp로 받은 것이 mu2 일때
    def handle_mu2(self,payload_dict):
        # 목적지 도착 알림
        robot_id = payload_dict.get("robot_id")
        print(f"[TCP 수신]: {robot_id}: 목적지 도착 알림 (AR)")
        x1, y1 = self.position #현재위치
        x0, y0 = (3.0, 13.0) #충전소 좌표
        distance =  ((x1 - x0)**2 + (y1 - y0)**2)**0.5 #현재위치와 충전소 사이 거리

        if distance > 0.5:
            arrived = self.destinations.pop(0)
            self.arrived_key = self.selected_keys.pop(0)
            self.arrived_place = arrived
            payload_to_send = {
                "robot_id": 1,
                "user_info": None,
                "des_coor": None
            }
            self.tcp.send_tcp_message_um("PS", payload_to_send)
            print("[TCP] PS 전송 완료")
            payload_to_send = {
                "robot_id": 1,
                "user_info": None,
                "des_coor": self.arrived_place
            }
            self.tcp.send_tcp_message_um("AR", payload_to_send)
            print("[TCP] AR 전송 완료")
            self._refresh_place_buttons() 
            print(f"AR수신받은 후 목적지(key): {self.selected_keys}")
            print(f"AR수신받은 후 목적지(des): {self.destinations}")
            print(f"새로 도착한 장소(key): {self.arrived_key}")
            print(f"새로 도착한 장소(des) : {self.arrived_place}")
            self.show_page('decision')
        else: 
            self.show_page('charging')

    #tcp로 받은 것이 mu3 일때
    def handle_mu3(self, payload_dict):
        payload_to_send = {
            "robot_id": 1,
            "user_info": None,
            "des_coor": None
        }
        self.tcp.send_tcp_message_um("PS", payload_to_send)
        llm_place_list = payload_dict.get("destination_location")
        llm_msg = payload_dict.get("response") or ""
        print(f"[TCP 수신] llm메시지: {llm_msg}")
        time.sleep(1)
        
        if self.selected_function == 'leading_mode':
            # llm_place_list = payload_dict.get("destination_location")
            if llm_place_list is not None: 
                llm_place_tuple = tuple(llm_place_list)
                llm_destination = (*llm_place_tuple, 0.0)
                #destinations에 중복 llm_destination 들어가지 않게 방지
                existing_coords = [(x, y) for (x, y, _) in self.destinations]
                if llm_place_tuple not in existing_coords:
                    self.destinations.insert(0, llm_destination)

                    llm_place_key = next((k for k, v in self.place_coords.items() if tuple(v) == llm_destination),None)
                    if llm_place_key is not None:
                        self.selected_keys.insert(0, llm_place_key)
                    else:
                        # (옵션) 만약 매핑이 없으면 예외 처리하거나 로그 남기기
                        print(f"[WARN] no key found for coords {llm_place_tuple}")
                    print(f"목적지 추가됨: {llm_destination}")

                else:
                    print(f"이미 존재하는 목적지, 추가 안함: {llm_destination}")

                print(f"가야 할 목적지: {self.destinations}")
                payload_to_send ={
                "robot_id": 1,
                "user_info": None,
                "des_coor": self.destinations}
                self.tcp.send_tcp_message_um("ED", payload_to_send)
                print("[TCP 전송] llm목적지 추가해서 ED 전송완료")
            
            else:
                payload_to_send = {
                        "robot_id": 1,
                        "user_info": None,
                        "des_coor": self.destinations
                    }
                self.tcp.send_tcp_message_um("LD", payload_to_send)
                print("[TCP] LD 전송 완료")


        elif self.selected_function == 'following_mode':
            if llm_place_list:
                llm_msg = "따라가는 기능 중에는 길 안내를 할 수 없어요."
                print(f"[TCP 수신] llm목적지 : {llm_place_list} / 안내 불가능")
            payload_to_send = {
            "robot_id": 1,
            "user_info": self.user_info,
            "des_coor": None
            }
            self.tcp.send_tcp_message_um("FW", payload_to_send)
            print("[TCP] FW 전송 완료")
        else:
            llm_msg = "나를 따라와줘 또는 길 안내해줘를 선택 후 아크를 불러주세요"
        
        self.play_tts_async(llm_msg) # 말하자


    #tcp로 받은 것이 mu1일때
    def handle_mu1(self, payload_dict):
        # print(f"[TCP 수신] OO payload_dict 업데이트")
        #payload_dict 에서 필요한 정보들 분리
        loadcell = payload_dict.get("loadcell")               #무게
        self.position = payload_dict.get("current_position")       #현재위치(x,y)
        distance_state = payload_dict.get("distance")   #사용자와의 거리 상태
        # print(f"받은 distance_state: {distance_state}")
        path = payload_dict.get("path")                       #목적지까지의 경로
        #실제 gui에서 사용
        if loadcell is not None:
            # print(f"받은 loadcell:{loadcell}")
            abs_loadcell = abs(loadcell)    #실험 일단 절댓값으로
            self._update_loadcell_ui(abs_loadcell)
        if self.position is not None:
            # print(f"받은 self.position: {self.position}")
            self._update_position_ui()
        if distance_state is not None:
            # if distance_state == 1:
                # print(f"받은 distance_state: {distance_state}")
            self._update_distance_text(distance_state)
        if path is not None:
            # print(f"받은 path : {path}")
            self.path = path   #새 path 들어오면 갱신
            self._update_map_ui(path) #맵 갱신
            if self.destinations:
                self.calculate_total_distance(path) #최종목적지까지 관련 계산
        #다음 목적지까지
        if self.path and self.destinations:      
            next_coord = self.destinations[0][:2]
            self.calculate_distance_to_next_destination(self.path, next_coord)

    def _update_loadcell_ui(self, weight):
        # print("_update_loadcell_ui함수 진입")
        #무게 weight_bar에 시각화
        for i in range(4, 13):
            bar = self.findChild(QProgressBar, f"weight_bar{i}")
            if bar:
                weight_g = int(weight*1000)
                clamped = min(weight_g, bar.maximum())
                bar.setValue(clamped)#단위 : g
                bar.setFormat(f"{weight:.2f}kg")

                if int(weight*1000) > bar.maximum():
                    bar.setStyleSheet('''
                                QProgressBar { 
                                      border:2px solid #000000; 
                                      background-color: #F0F0F0; 
                                      text-align: center;               
                                    }
                                QProgressBar::chunk {
                                      background-color: red; 
                                      }''')
                else:
                    bar.setStyleSheet('''
                                QProgressBar { 
                                      border:2px solid #000000; 
                                      background-color: #F0F0F0; 
                                      text-align: center;               
                                    }
                                QProgressBar::chunk {
                                      background-color: rgba(129, 224, 144, 180); 
                                      }''')
            

    def _update_position_ui(self):
        # print("_update_position_ui 함수 진입")
        try:
            x,y = self.position
            if self.current_page in ['select_place', 'check_goal', 'guiding']:
                self.update_arcs_position(x,y)
        except Exception:
            print("[현재 위치 업데이트 오류]")

    def _update_distance_text(self, state):
        # print("_update_distance_text 함수 진입")
        if self.current_page == 'following' and hasattr(self, 'label_following_text5'): #해당 페이지에 label_following_text5가 있으면
            if state == 0: #정상
                if self.before_state_f == 1:
                    self._stop_tts()
                    msg = "오케이, 다시 출발하셔도 될 것 같아요!"
                    self.play_tts_async(msg)
                self.label_following_text5.setText("동행중이에요:)")
                self.before_state_f = state
            else:  # state == 1인 경우 (정해진 거리보다 멀어졌을 때)
                self.before_state_f = state
                msg = "우리 사이가 조금 멀어진 것 같아요. 조금만 기다려줄래요?"
                self.play_tts_async(msg)
                self.label_following_text5.setText("조금만 기다려주세요, 가고 있어요!")

        if self.current_page == 'guiding' and hasattr(self, 'label_check_place_text8'): #해당 페이지에 label_check_place_text8 있으면
            if state == 0: #정상
                if self.before_state_g == 1:
                    self._stop_tts()
                    msg = "다시 출발하겠습니다"
                    self.play_tts_async(msg)
                    payload_to_send = {
                        "robot_id": 1,
                        "user_info": None,
                        "des_coor": self.destinations
                    }
                    self.tcp.send_tcp_message_um("LD", payload_to_send)
                self.label_check_place_text8.setText("목적지로 길 안내중이에요:)")
                self.before_state_g = state
            else:  # state == 1인 경우 (정해진 거리보다 멀어졌을 때)
                self.before_state_g = state
                payload_to_send = {
                    "robot_id": 1,
                    "user_info": None,
                    "des_coor": None
                }
                self.tcp.send_tcp_message_um("PS", payload_to_send)
                msg = "어디갔어요? 여기서 기다릴테니 얼른 다가와주세요"
                self.play_tts_async(msg)
                self.label_check_place_text8.setText("잘 따라와 주세요 :)")

    def _update_map_ui(self, path):
        # print("_update_map_ui 함수 진입")
        if self.current_page in ['check_goal', 'guiding']:
            self.draw_path(path)

    #최종 목적지까지
    def calculate_total_distance(self, path):
        total_dist = 0.0
        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            total_dist += ((x2 - x1)**2 + (y2 - y1)**2)**0.5
        
        total_dist = round(total_dist, 2)
        print(f"total = {total_dist}")
        predicted_time_s = round(total_dist / 0.2)
        print(f"predicted_time_s : {predicted_time_s}")
        minutes = predicted_time_s // 60
        seconds = predicted_time_s % 60
        predicted_time_text = f"{minutes}분 {seconds}초" if minutes else f"{seconds}초"
        if hasattr(self, 'label_predicted_time7'):
            self.label_predicted_time7.setText(predicted_time_text)

    #다음 목적지까지
    def calculate_distance_to_next_destination(self, path, next_goal):
        tolerance = 0.25
        next_goal_dist = 0.0
        # 현재 위치
        x0, y0 = self.position
        # path 중 현재 위치에서 가장 가까운 점의 인덱스 찾기
        min_idx = 0
        min_dist = float('inf')
        for i, (x, y) in enumerate(path):
            dist = ((x - x0) ** 2 + (y - y0) ** 2) ** 0.5
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        # 현재 위치부터 path[min_idx + 1]부터 누적 거리 계산
        for i in range(min_idx, len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            segment_dist = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
            next_goal_dist += segment_dist
            # 목적지에 가까워졌다면 종료
            if ((x2 - next_goal[0]) ** 2 + (y2 - next_goal[1]) ** 2) ** 0.5 < tolerance:
                break
        #다음 목적지까지 남은 거리 page_guiding에 띄우기 (속도: 0.2m/s 기준)
        next_goal_dist = round(next_goal_dist, 2)
        # print(f"next_dist = {next_goal_dist}")
        if hasattr(self, 'label_remained_distance8'):
            self.label_remained_distance8.setText(f"{next_goal_dist}m")
        #다음 목적지까지 남은 시간 page_guiding에 띄우기
        remained_time_s = round(next_goal_dist / 0.2)
        # print(f"remained_time : {remained_time_s}")
        minutes = remained_time_s // 60
        seconds = remained_time_s % 60
        remained_time_text = f"{minutes}분 {seconds}초" if minutes else f"{seconds}초"
        if hasattr(self, 'label_remained_time8'):
            self.label_remained_time8.setText(remained_time_text)

    def real_to_scene(self, x_m, y_m):
        #월드 좌표를 맵 픽셀로 변환
        map_x = (x_m - self.origin[0]) / self.resolution
        map_y = (-y_m - self.origin[1]) / self.resolution
        #회전 보정 (90도 회전)
        scene_x = self.map_image.height() - map_y
        scene_y = map_x
        #위치 보정 (예: 원점이 지도에서 조금 위쪽으로 있어야 함)
        offset_x = -12     # 좌우 보정
        offset_y = -5    # 상하 보정

        x = scene_x + offset_x
        y = scene_y + offset_y
        return x,y

    #로봇 현재 위치 업데이트
    def update_arcs_position(self, x_m, y_m):
        x,y = self.real_to_scene(x_m, y_m)
        self.arcs_item.setPos(x,y) #현재 위치 맵에 시각화

    #실좌표 받아서 맵좌표로 변환 후 경로 그리기
    def draw_path(self, path_points):
        # 기존 선 제거
        for item in list(self.scene.items()):
            if isinstance(item, QGraphicsLineItem):
                self.scene.removeItem(item)
        self.path_items = []

        try:
            for i in range(len(path_points) - 1):
                # 실좌표 → scene 좌표
                x1_m, y1_m = path_points[i]
                x2_m, y2_m = path_points[i + 1]

                x1, y1 = self.real_to_scene(x1_m, y1_m)
                x2, y2 = self.real_to_scene(x2_m, y2_m)

                # 선 그리기
                line = QGraphicsLineItem(x1, y1, x2, y2)
                line.setPen(QPen(Qt.blue, 1))
                self.scene.addItem(line)
                self.path_items.append(line)
        except Exception as e:
            print("[경로 그리기 오류]", e)

# ===========================================================================
    # 1) page_main (main)
# ===========================================================================
    #버튼 연결
    def _connect_page_main(self):
        self.btn_kor1.clicked.connect(self._to_korean)
        self.btn_eng1.clicked.connect(self._to_english)
        self.btn_follow1.clicked.connect(lambda: self._select_and_scan('following_mode'))
        self.btn_guide1.clicked.connect(lambda: self._select_and_scan('leading_mode'))

    #main_page 들어갔을 때 하는 일
    def _enter_main(self):
        self.reset_all()
        msg = "반가워요, 저는 아크에요! 아래에서 필요한 기능을 선택해주세요"
        self.play_tts_async(msg)
        # print(f"page_main : {self.selected_function}")

    #following 또는 leading 중에 뭐 눌렀는 지 저장
    def _select_and_scan(self, mode):
        payload_to_send = {
            "robot_id": 1,
            "user_info": None,
            "des_coor": None
        }
        self.tcp.send_tcp_message_um("PS", payload_to_send)
        print("[TCP] PS 전송 완료")
        self.selected_function = mode
        if self.selected_function == 'following_mode':
            msg = "넵 따라갈게요! 궁금한 게 있을 때 질문 앞에 아크야를 붙여서 질문해주시면 친절히 답 드릴게요"
        else:
            msg = "저만 믿으세요! 원하시는 목적지까지 잘 데려다 드릴게요"
        self.play_tts_async(msg)
        self.show_page('scan')

    #한글 모드로
    def _to_korean(self):
        if getattr(self, 'language_mode', 'kor') != 'kor':
            self.language_mode = 'kor'
            # TODO: 전체 UI를 한글로 변환
            
    #영어 모드로
    def _to_english(self):
        if getattr(self, 'language_mode', 'kor') == 'kor':
            self.language_mode = 'eng'
            # TODO: 전체 UI를 영어로 변환
            
# ===========================================================================
    # 2) page_scan (scan)
# ===========================================================================
    #버튼 연결
    def _connect_page_scan(self):
        self.btn_back2.setEnabled(False)

    #page_scan들어갔을 때 하는 일
    def _enter_scan(self):
        msg = "전방 카메라에 탑승권을 보여주세요!"
        self.play_tts_async(msg)
        print(f"page_scan: {self.selected_function}")
        self.camera.start()

    #page_scan 나올 때 하는 일
    def _exit_scan(self):
        self.camera.stop()

    #카메라 프레임,qr이미지 출력 및 가이드박스 그리기
    def on_camera_frame(self, frame: QImage):
        if self.current_page != 'scan':
            return
        # PyQt에서 받은 QImage를 OpenCV가 이해할 수 있는 numpy 배열로 바꾸기 위한 전처리 과정
        width = frame.width()
        height = frame.height()
        channels = 4 if frame.hasAlphaChannel() else 3 #투명도 채널있으면(rgba) 4 없으면(rgb) 3
        ptr = frame.bits()
        ptr.setsize(height * width * channels)
        np_img = np.array(ptr, dtype=np.uint8).reshape((height, width, channels))
        if channels == 4:
            np_img = cv2.cvtColor(np_img, cv2.COLOR_RGBA2BGR)
        else:
            np_img = cv2.cvtColor(np_img, cv2.COLOR_RGB2BGR)
        #가이드 박스 설정 (중앙 정사각형)
        guide_size = 200
        cx, cy = width // 2, height // 2
        x1, y1 = cx - guide_size // 2, cy - guide_size // 2
        x2, y2 = cx + guide_size // 2, cy + guide_size // 2
        cv2.rectangle(np_img, (x1, y1), (x2, y2), (255, 0, 0), 2)
        #numpy → QImage
        rgb = cv2.cvtColor(np_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()
        #QImage → QLabel
        pixmap = QPixmap.fromImage(qimg)
        self.label_camera_area2.setPixmap(pixmap.scaled(
            self.label_camera_area2.width(),
            self.label_camera_area2.height(),
            # Qt.KeepAspectRatio,
            Qt.IgnoreAspectRatio,
            Qt.SmoothTransformation
        ))

    #qr이미지에서 정보 얻어와서 사용할 수 있게 제작, 비밀번호 설정
    def _on_qr_detected(self, data):
        #qr 정보 얻기
        self.scanned_data = data.split(',')
        print(f"scanned_data : {self.scanned_data}")
        # 탑승권 번호 마지막 4자리 correct_password로 저장.
        last4 = str(self.scanned_data[0])[-4:]
        self.correct_password = f"#{last4}*"
        print(self.correct_password)
        # 얻을 거 다 얻으면 카메라 끄고 다음 페이지로
        if data:
            self.camera.stop()                 # 카메라 멈추고
            self.show_page('check_info')       # 다음 페이지로

# ===========================================================================
    # 3) page_check_info (check_info)
# ===========================================================================
    #버튼 연결
    def _connect_page_check_info(self):
        # 확인 버튼: loading으로 이동
        self.btn_correct3.clicked.connect(self._go_loading)
        # 재스캔 또는 뒤로 버튼: 스캔 데이터 리셋 후 scan으로 이동
        self.btn_retry3.clicked.connect(self._on_retry3)
        self.btn_back3.clicked.connect(self._on_retry3)

    #qr 정보 테이블위젯에 채우기
    def _enter_check_info(self):
        # print(f"page_check_info: {self.selected_function}")
        msg = "제가 인식한 정보가 맞는지 확인 부탁드려요"
        self.play_tts_async(msg)
        keys = [
            'ticket',  #탑승권번호
            'name',    #이름
            'age',     #나이
            'sex',     #성별
            'from',    #출국지
            'to',      #입국지
            'seat',    #좌석
            'gate',    #탑승게이트
            'boarding',#탑승시간
            'departure'#출발시간
        ]
        
        for i, val in enumerate(self.scanned_data):
            # 스캔된 데이터로 테이블 채우기
            self.tableWidget3.setItem(i, 1, QTableWidgetItem(str(val)))
            # self.user_info 생성
            if i < len(keys):
                self.user_info[keys[i]] = str(val)

    #다시 스캔해야 할 경우
    def _on_retry3(self):
        self.scanned_data = []
        self.user_info = {} 
        #두 번째 열 내용만 지우기
        self.clear_second_column_only()
        #해당 페이지로 이동
        self.show_page('scan')

    #두번째 열 내용 제거
    def clear_second_column_only(self):
        for row in range(self.tableWidget3.rowCount()):
            item = self.tableWidget3.item(row, 1)  # 두 번째 열 (index 1)
            if item is not None:
                item.setText("")  # 내용만 지우기

    #무게측정하라고 명령 보내기
    def _go_loading(self):
        payload_to_send = {
            "robot_id": 1,
            "user_info": None,
            "des_coor": None
        }
        self.tcp.send_tcp_message_um("KG", payload_to_send)
        print("[TCP] KG 전송 완료")
        self.show_page('loading')

# ===========================================================================
    # 4) page_loading(loading)
# ===========================================================================
    #버튼 연결
    def _connect_page_loading(self):
        self.btn_end_loading4.clicked.connect(self._route_from_loading)
        self.btn_deny_load4.clicked.connect(self._route_from_loading)
        self.btn_back4.clicked.connect(lambda: self.show_page('check_info'))

    #page_loading 들어갔을 때 하는 일
    def _enter_loading(self):
        if self.before_page == "check_info":
            msg = "짐을 저에게 보관해주세요! 20kg가 넘으면 출발을 못 하니 주의해주세요"
            self.play_tts_async(msg)
        # print(f"page_loading: {self.selected_function}")
       

    #page_loading에서 나갈 때 하는 일
    def _exit_loading(self):
        pass
    
    #다음 페이지로 넘어갈 때 조건 분기
    def _route_from_loading(self):
        if self.selected_function == 'following_mode':
            payload_to_send = {
            "robot_id": 1,
            "user_info": self.user_info,
            "des_coor": None
            }
            self.tcp.send_tcp_message_um("FW", payload_to_send)
            print("[TCP] FW 전송 완료")
            self.show_page('following')
        #self.selected_function == 'leading_mode'일 때
        else: 
            self.show_page('select_place')

# ===========================================================================
    # 5) page_following
# ===========================================================================
    #버튼 연결
    def _connect_page_following(self):
        self.btn_back5.setEnabled(False)
        self.btn_end_following5.clicked.connect(lambda: self.show_page('bye'))
        self.btn_pause5.clicked.connect(self._toggle_following_pause)

    #page_following 들어갔을 때 하는 일
    def _enter_following(self):
        msg = "잘 따라갈 테니 걱정마세요! 거리가 멀어지면 말씀 드릴게요"
        self.play_tts_async(msg)
        # print(f"page_following: {self.selected_function}")
        

    #page_following 나올 때 하는 일
    def _exit_following(self):
        pass
        
    #btn_pause 텍스트 변경
    def _toggle_following_pause(self):
        #일시정지 눌렀을 때
        if not self.is_paused_f:
            payload_to_send = {
            "robot_id": 1,
            "user_info": None,
            "des_coor": None
            }
            self.tcp.send_tcp_message_um("PS", payload_to_send)
            print("[TCP] PS 전송 완료")
            msg = "일시정지를 눌렀네요"
            self.play_tts_async(msg)
            self.btn_pause5.setText("주행시작")
            self.is_paused_f = True
            self.show_page("waiting")
        #주행시작 눌렀을 때
        else:
            payload_to_send = {
            "robot_id": 1,
            "user_info": self.user_info,
            "des_coor": None
            }
            self.tcp.send_tcp_message_um("FW", payload_to_send)
            print("[TCP] FW 전송 완료")
            msg = "다시 잘 따라갈게요"
            self.play_tts_async(msg)
            self.btn_pause5.setText("일시정지")
            self.is_paused_f = False

# ===========================================================================
    # 6) page_select_place
# ===========================================================================
    #버튼 연결
    def _connect_page_select_place(self):
        for btn in self.findChildren(QPushButton):
            if btn.objectName().startswith('btn_place_'):
                btn.clicked.connect(lambda _,b=btn: self._toggle_destination(b))
        self.btn_back6.clicked.connect(self._go_back_page)
        self.btn_start6.clicked.connect(self._check_goal) 

    #page_select_place 들어갔을 때 하는 일
    def _enter_select_place(self):
        if self.before_page == 'loading':
            msg = "가고싶은 목적지를 순서대로 선택해주세요. 버튼을 다시 누르시면 취소 가능해요"
            self.play_tts_async(msg)
        # → 경로 아이템 삭제(해당 페이지에는 경로 그리지 않도록)
        for item in list(self.scene.items()):
            if isinstance(item, QGraphicsLineItem):
                self.scene.removeItem(item)
        self.path_items = []
        # print(f"page_select_place: {self.selected_function}")
        self.view_map6.setScene(self.scene)  #맵에 현재 위치 시각화
        # 기존 변형 초기화
        self.view_map6.resetTransform()
        # 동훈님 방법====
        self.view_map6.scale(3.0, 3.0)
        #============

    #page_select_place 나올 때 하는 일
    def _exit_select_place(self):
        pass
    
    #btn_back 눌렀을 때 (초기화)
    def _go_back_page(self):
        self.destinations = []
        self.selected_keys = []
        for btn in self.findChildren(QPushButton):
            if btn.objectName().startswith("btn_place_"):
                btn.setStyleSheet("")
        self.show_page('loading')

    #btn_place_* 누를 때마다 일어나는 일
    def _toggle_destination(self, btn):
        key = btn.objectName().split('btn_place_')[1]
        coords = self.place_coords.get(key)
        if not coords:
            return
        #selected_keys에 해당 목적지 들어있으면
        if key in self.selected_keys:                 
            idx = self.selected_keys.index(key)
            self.selected_keys.pop(idx)
            self.destinations.pop(idx)
            btn.setStyleSheet("")  
        #selected_keys에 해당 목적지 들어있지 않으면
        else:
            self.selected_keys.append(key)
            self.destinations.append(coords)
            btn.setStyleSheet("background-color: lightblue;")
        print(f"목적지 선택중(key): {self.selected_keys}")
        print(f"목적지 선택중(des): {self.destinations}")
        print(f"도착한 장소: {self.arrived_place}")

    #page_selected_place의 장소버튼 색깔 변경
    def _refresh_place_buttons(self):
        for key in self.place_coords:
            btn = self.findChild(QPushButton, f'btn_place_{key}')
            if not btn:
                continue
            if key in self.selected_keys:
                btn.setStyleSheet("background-color: lightblue;")
            else:
                btn.setStyleSheet("")

    def _check_goal(self):
        payload_to_send = {
            "robot_id": 1,
            "user_info": self.user_info,
            "des_coor": self.destinations
        }
        self.tcp.send_tcp_message_um("CK", payload_to_send)
        print("[TCP] CK 전송 완료")
        self.show_page("check_goal")
    


# ===========================================================================
    # 7) page_check_goal
# ===========================================================================
    #버튼 연결
    def _connect_page_check_goal(self):
        self.btn_start7.clicked.connect(self._start_guide)
        self.btn_change_place7.clicked.connect(self._change_place_from_check_goal)
        self.btn_back7.setEnabled(False)

    #들어갈 때 하는 일
    def _enter_check_goal(self):
        place_text = ' '.join(self.selected_keys)
        msg = f"{place_text}를 선택했네요. 경로를 확인해주세요"
        self.play_tts_async(msg)
        print(f"page_check_goal: {self.selected_function}")
        print(f"선택된 목적지(key): {self.selected_keys}")
        print(f"선택된 목적지(des): {self.destinations}")
        print(f"도착한 장소: {self.arrived_place}")
        self.view_map7.setScene(self.scene)  #맵에 현재 위치, 경로 시각화
        #스케일 했던 거 갱신
        self.view_map7.resetTransform()
        #==================
        self.view_map7.scale(3.0, 3.0) 
        #==================

    #나올 때 하는 일
    def _start_guide(self):
        #목적지 편집 상태가 아니면
        if not self.edit:  
            payload_to_send = {
                "robot_id": 1,
                "user_info": None,
                "des_coor": self.destinations
            }
            self.tcp.send_tcp_message_um("LD", payload_to_send) # um_5 전송
            print("[TCP] LD 전송 완료")
        #목적지 편집상태면
        else: 
            payload_to_send = {
                "robot_id": 1,
                "user_info": None,
                "des_coor": self.destinations
            }
            self.tcp.send_tcp_message_um("ED", payload_to_send) # um_7 전송
            print("[TCP] ED 전송 완료")
            self.edit = False
        self.show_page('guiding')

    def _change_place_from_check_goal(self):
        msg = "가고 싶은 목적지를 다시 순서대로 선택해주세요"
        self.play_tts_async(msg)
        self.edit = True
        self.show_page("select_place")

# ===========================================================================
    # 8) page_guiding
# ===========================================================================
    #버튼 연결
    def _connect_page_guiding(self):
        self.btn_back8.setEnabled(False)
        self.btn_pause8.clicked.connect(self._toggle_guiding_pause)
        self.btn_change_place8.clicked.connect(self._change_place_from_guiding)

    #들어갈 때 하는 일
    def _enter_guiding(self):
        
        msg = f"{self.selected_keys[0]}까지 안내해 드릴게요. 저를 따라오세요"
        self.play_tts_async(msg)
        print(f"page_guiding: {self.selected_function}")
        print(f"진짜 선택된 목적지(key): {self.selected_keys}")
        print(f"진짜 선택되 목적지(des): {self.destinations}")
        print(f"도착한 장소: {self.arrived_place}")
        self.view_map8.setScene(self.scene)  #맵에 현재 위치, 경로 시각화

        #스케일 했던 거 갱신
        self.view_map8.resetTransform()
        #==========
        self.view_map8.scale(3.0, 3.0)
        # =========

    #나올 때 하는 일
    def _exit_guiding(self):
        pass

    #btn_pause 텍스트 변경
    def _toggle_guiding_pause(self):
        if not self.is_paused_g: #일시정지 눌렀을 때
            self.is_paused_g = True
            payload_to_send = {
                "robot_id": 1,
                "user_info": None,
                "des_coor": None
            }
            self.tcp.send_tcp_message_um("PS", payload_to_send)
            print("[TCP] PS 전송 완료")
            msg = "일시정지를 눌렀네요"
            self.play_tts_async(msg)
            self.btn_pause8.setText("이어서 안내시작")
            self.show_page("user_valid")
        else: #이어서 안내시작 눌렀을 때
            self.is_paused_g = False
            payload_to_send = {
            "robot_id": 1,
            "user_info": None,
            "des_coor": self.destinations
            }
            self.tcp.send_tcp_message_um("LD", payload_to_send)
            print("[TCP] LD 전송 완료")
            self.btn_pause8.setText("일시정지")

    # 목적지 변경 클릭 시
    def _change_place_from_guiding(self):
        payload_to_send = {
            "robot_id": 1,
            "user_info": None,
            "des_coor": None
        }
        self.tcp.send_tcp_message_um("PS", payload_to_send)
        print("[TCP] PS 전송 완료")
        msg = "가고 싶은 목적지를 다시 순서대로 선택해주세요"
        self.play_tts_async(msg)
        self.edit = True
        self.show_page("select_place")

# ===========================================================================
    # 9) page_decision_after_arrived
# ===========================================================================
    #버튼 연결
    def _connect_page_decision(self):
        self.btn_back9.setEnabled(False)
        self.btn_go_next_goal9.clicked.connect(self._go_next_goal)  
        self.btn_change_goal9.clicked.connect(self._change_place_from_decision)
        self.btn_wait9.clicked.connect(self._wait_here)
        self.btn_bye9.clicked.connect(lambda: self.show_page('bye'))
    
    #page_decision_after_arrived 들어갈 때 하는 일
    def _enter_decision(self):
        if self.before_page == "guiding":
            msg = f"{self.arrived_key}에 도착했어요. 이제 무엇을 할 지 선택해주세요"
        else:
            msg = "이제 무엇을 할 지 선택해주세요"
        self.play_tts_async(msg)
        print("I arrived")
        print(f"page_decision_after_arrived: {self.arrived_key}")

        self.decision_timer.start(20000) #들어오고 20초 반응 없으면 rt쏘고 복귀
        if not self.destinations:
            return

    def _exit_decision(self):
       # decision 페이지를 벗어날 때 타이머 자동 취소
        if hasattr(self, 'decision_timer') and self.decision_timer.isActive():
            self.decision_timer.stop()

    #다음 목적지로 계속 이동
    def _go_next_goal(self):
        if hasattr(self, 'decision_timer') and self.decision_timer.isActive():
            self.decision_timer.stop()
            self.show_page("decision")

        if self.destinations:
            self.btn_go_next_goal9.setText("다음 목적지 가자")
            payload_to_send = {
                "robot_id": 1,
                "user_info": None,
                "des_coor": self.destinations
            }
            print(f"des : {self.destinations}")
            self.tcp.send_tcp_message_um("LD", payload_to_send) 
            print("[TCP] LD 전송 완료")
            self.show_page("guiding")
        else:
            self.btn_go_next_goal9.setText("다음 목적지 없어요")
            msg = "선택하신 목적지들을 다 안내해드렸어요. 가고싶은 곳이 추가로 있으시다면 목적지 변경할래를 선택해주시고 아니라면 수고했어를 눌러주세요."
            self.play_tts_async(msg)

    #목적지 변경 선택 시
    def _change_place_from_decision(self):
        msg = "가고 싶은 목적지를 다시 순서대로 선택해주세요"
        self.play_tts_async(msg)
        self.edit = True
        self.show_page("select_place")

    #여기서 기다려 선택 시
    def _wait_here(self):
        payload_to_send = {
            "robot_id": 1,
            "user_info": None,
            "des_coor": None
        }
        self.tcp.send_tcp_message_um("PS", payload_to_send)  # um_4
        print("[TCP] PS 전송 완료")
        self.show_page("waiting")

    def _on_decision_timeout(self):
        # 아직 decision 페이지에 머물러 있을 때만 동작
        if self.current_page != "decision":
            return

        # TTS 안내
        self.play_tts_async("반응이 없어 충전소로 자동복귀할게요")
        payload_to_send = {
            "robot_id": 1,
            "user_info": None,
            "des_coor": [(3.0,13.0,270.0)] #충전소 좌표
        }
        self.tcp.send_tcp_message_um("RT", payload_to_send)
        self.show_page("main")
        print("[TCP] RT 전송 완료 (타임아웃)")

# ===========================================================================
    # 10) page_waiting
# ===========================================================================
    #버튼 연결
    def _connect_page_waiting(self):
        self.btn_back10.clicked.connect(lambda: self.show_page('user_valid'))
        self.btn_imback10.clicked.connect(lambda: self.show_page('user_valid'))

    #page_waiting 들어갈 때 하는 일
    def _enter_waiting(self):
        msg = "네 여기서 기다릴게요! 돌아오시면 나 왔어 버튼을 눌러주세요"
        self.play_tts_async(msg)
        # print(f"page_waiting: {self.selected_function}")


# ===========================================================================
    # 11) page_user_valid
# ===========================================================================
    #버튼 연결
    def _connect_page_user_valid(self):
        self.btn_back11.clicked.connect(lambda: self.show_page('waiting'))
        # '*' → 'star', '#' → 'sharp' 매핑
        special = {'*': 'star', '#': 'sharp'}
        for key in ['1','2','3','4','5','6','7','8','9','0','*','#']:
            attr = special.get(key, key)               # '*'이면 'star', '#'이면 'sharp'
            btn = getattr(self, f'btn_{attr}')         # btn_star, btn_sharp
            btn.clicked.connect(lambda _, k=key: self._handle_user_input(k))

    #page_user_valid 들어갈 때 하는 일
    def _enter_user_valid(self):
        # print(f"page_user_valid: {self.selected_function}")
        self.pushed_valid_btn = ''
        msg = "비밀번호를 입력해주세요. 비밀번호는 탑승권 마지막 4자리입니다."
        self.play_tts_async(msg)
        self.label_ps_text11.setText('돌아오셨나요? 비밀번호를 입력해주세요')

    #비밀번호키 입력값 기억, 비교, 결과
    def _handle_user_input(self, key):
        self.pushed_valid_btn += key
        if key == '*':
            if self.pushed_valid_btn == self.correct_password:
                if self.selected_function == 'leading_mode':
                    if self.before_page  == 'guiding':
                        self.show_page('guiding')
                    else:
                        self.show_page('decision')
                else:
                    self.show_page('following')
            else:
                msg ="비밀번호가 틀렸습니다. 다시 입력해주세요"
                self.play_tts_async(msg)
                self.label_ps_text11.setText("인증 실패")
                self.pushed_valid_btn = ''

# ===========================================================================
    # 12) page_bye
# ===========================================================================
    def _connect_page_bye(self):
        self.btn_back12.setEnabled(False)

    def _enter_bye(self):
        print(f"page_bye: {self.selected_function}")
        payload_to_send = {
            "robot_id": 1,
            "user_info": None,
            "des_coor": None
        }
        self.tcp.send_tcp_message_um("PS", payload_to_send)
        print("[TCP] PS 전송 완료")
        msg = "도움이 되어 기뻤어요. 행복한 여행 되세요"
        self.play_tts_async(msg)

        self.blink = False
        self.bye_timer = QTimer(self)
        self.bye_timer.timeout.connect(self._blink_bye)
        self.bye_timer.start(1000)
        QTimer.singleShot(6000, self._send_um3_and_go_home)

    def _exit_bye(self):
        self.bye_timer.stop()

    def _blink_bye(self):
        self.blink = not self.blink
        for lbl in [self.label_bye_icon12, self.label_bye_text12, self.label_bye_text12_2]:
            lbl.setVisible(self.blink)

    def _send_um3_and_go_home(self):
        payload_to_send = {
            "robot_id": 1,
            "user_info": None,
            "des_coor": [(3.0,13.0,270.0)] #충전소 좌표
        }
        self.tcp.send_tcp_message_um("RT", payload_to_send)
        print("[TCP] RT 전송 완료")
        msg = "충전소로 복귀할게요"
        self.play_tts_async(msg)
        self.show_page("main")

# ===========================================================================
    # 13) page_charging  #지금 안 쓰이고 있음. 빠때리 몇 이하일 때 행동 조건 만들기
# ===========================================================================
    def _connect_page_charging(self):
        pass

    def _enter_charging(self):
        msg = "충전소로 복귀완료했어요."
        self.play_tts_async(msg)
        print(self.selected_function)
        self.charge_blink = False
        self.charge_timer = QTimer(self)
        self.charge_timer.timeout.connect(self._blink_charge)
        self.charge_timer.start(500)
        self.battery = 30
        self.batt_timer = QTimer(self)
        self.batt_timer.timeout.connect(self._update_battery)
        self.batt_timer.start(1000)

    def _exit_charging(self):
        self.charge_timer.stop()
        self.batt_timer.stop()

    def _blink_charge(self):
        self.charge_blink = not self.charge_blink
        self.label_charge13.setVisible(self.charge_blink)

    def _update_battery(self):
        self.battery += 1
        self.label_battery13_2.setText(f"{self.battery}%")
        if self.battery >= 90:
            self.batt_timer.stop()
    
# ==================================================================================
#  닫아
# ==================================================================================   
    def closeEvent(self, event):   # Qt 프레임워크가 메인 윈도우가 닫힐 때 자동으로 실행하는 이벤트 핸들러
        self._stop_tts()
        self._tts_worker_running = False
        self._tts_worker.join(timeout=1.0) # 최대 1초 기다리고 넘어감
        #CameraThread 안전 종료
        if hasattr(self, 'camera'):
            self.camera.stop()  # threading.Thread.join() 호출로 안전 종료

        if hasattr(self, 'tcp'):
            self.tcp.close()

        event.accept() # Qt의 이벤트 시스템에 **“이 CloseEvent를 내가 처리했고, 윈도우를 닫아도 좋다”**라고 알려 주는 호출


if __name__ == '__main__':
    app = QApplication(sys.argv) #Qt 애플리케이션 객체 생성 : 내가 designer에서 만든 것 대로 윈도우에 배치해줌.
    win = MainWindow()  #mainwindow 클래스의 __init__ 함수를 즉시 실행.
    win.show() #메인 윈도우를 화면에 띄움
    sys.exit(app.exec_()) #app.exec_()가 호출되는 순간, qt는 내부적으로 "이제부터 gui 관련 모든 이벤트를 처리할 준비가 됐다"하고 무한루프 돌기 시작
                          #그 안에서 버튼클릭, 화면 그리기, 시그널/슬롯 호출, qtimer등 모든 qt 이벤트가 처리됨.
                          #윈도우를 닫으면 exec_()가 빠져나오고, 그 값을 sys.exit()로 넘겨서 스크립트가 마무리됩니다. 