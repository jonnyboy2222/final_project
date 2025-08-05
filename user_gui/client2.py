import sys
import cv2
import datetime
import socket
import json
import re
import struct
import threading
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLabel, QPushButton, QTableWidgetItem
)
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, Qt
from PyQt5.uic import loadUi
from PyQt5.QtGui import QImage, QPixmap

# Camera thread with QR detection
class CameraThread(QThread):
    # 카메라 프레임을 QImage로 전달하기 위한 시그널
    frame_received = pyqtSignal(QImage)
    # QR 코드가 인식되면 문자열을 전달하기 위한 시그널
    qr_detected   = pyqtSignal(str)

    def __init__(self, index=0):
        super().__init__()  
        self.index    = index                       
        self.running  = False                        
        self.detector = cv2.QRCodeDetector()         # QR 코드 디텍터 초기화

    def run(self):
        cap = cv2.VideoCapture(self.index)           
        self.running = True                          
        while self.running:                          
            ret, frame = cap.read()                  
            if not ret:                              
                continue
            # --- QR 코드 인식 파트 ---
            data, bbox, _ = self.detector.detectAndDecode(frame)  #  프레임에서 QR 디코딩
                                                       
            if data:                                 
                self.qr_detected.emit(data)         # 인식된 str을 UI로 전송
                self.running = False                 

            # --- 화면 표시용 이미지 변환 파트 ---
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                                                    
            h, w, ch = rgb.shape                     # 이미지 높이, 너비, 채널 수 추출
            # 원시 RGB 픽셀 데이터를 QImage로 변환
            img = QImage(rgb.data, w, h, ch*w, QImage.Format_RGB888).copy()
            self.frame_received.emit(img)            #  QImage를 UI로 전송

        cap.release()                               

    def stop(self):
        self.running = False
        self.quit()
        self.wait()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        loadUi("/home/dong/project_local/final_project/user_gui/client.ui", self)
        #state
        self.selected_function = None
        #인증번호 관련
        self.correct_password = ""
        self.pushed_valid_btn = ""
        #목적지 설정 관련
        self.place_coords ={             # 목적지 좌표 설정(임의값) - 맵 기반 좌표,pose 집어넣어야 함.
            'gate6': (1.0, 2.0, 0.0),    
            'shop6': (2.5, 3.0, 1.57),
            'toilet6': (3.0, 1.2, -1.57),
            'wait6': (0.5, 0.5, 3.14)
        }
        self.destinations = []  #a*에서 쓰일 목적지
        self.selected_keys = [] #page_selected_place에서 버튼 색 조절하려고 사용
        self.arrived_place = [] #도착한 장소 누적
        #가이드 중 일시정지 사용 위해서 
        self.is_paused_g = False
        #팔로잉 중 일시정지 사용 위해서
        self.is_paused_f = False
        # camera
        self.camera = CameraThread(0)
        self.scanned_data = []
        self.camera.frame_received.connect(self.on_camera_frame)
        self.camera.qr_detected.connect(self._on_qr_detected)

    #각 페이지에서 실행할 Qtimer 컨트롤 위해서
        # 공통 타이머 준비
        self._transition_timer = QTimer(self)
        self._transition_timer.setSingleShot(True)

        # 예약 메서드
        def schedule_transition(page_key, delay_ms):
            # 기존 예약이 있으면 끊고
            if self._transition_timer.isActive():
                self._transition_timer.stop()
            # 타이머가 만료되면 show_page(page_key) 실행
            #  이전 커넥션은 모두 끊어야 중복 호출 방지
            try: 
                self._transition_timer.timeout.disconnect()
            except TypeError: 
                pass
            self._transition_timer.timeout.connect(lambda: self.show_page(page_key))
            self._transition_timer.start(delay_ms)
        
        self._schedule_transition = schedule_transition

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
        #메인으로
        self.show_page('main')

    # 페이지 전환 할 때 쓰이는 함수
    def show_page(self, page_key):
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

# ===========================================================================
    # 1) page_main
# ===========================================================================
    #버튼 연결
    def _connect_page_main(self):
        self.btn_kor1.clicked.connect(self._to_korean)
        self.btn_eng1.clicked.connect(self._to_english)
        self.btn_follow1.clicked.connect(lambda: self._select_and_scan('following_mode'))
        self.btn_guide1.clicked.connect(lambda: self._select_and_scan('leading_mode'))

    #main_page 들어갔을 때 하는 일
    def _enter_main(self):
        print(self.selected_function)
        #초기화 관련 함수
        #스캔 관련
        self.scanned_data = []
        #선택한 기능 관련
        self.selected_function = None
        #인증번호 관련
        self.correct_password = ""
        self.pushed_valid_btn = ""
        #목적지 관련
        self.destinations = []  
        self.selected_keys = [] 
        self.arrived_place = [] 
        for btn in self.findChildren(QPushButton):
            if btn.objectName().startswith("btn_place_"):
                btn.setStyleSheet("")

    #following 또는 leading 중에 뭐 눌렀는 지 저장
    def _select_and_scan(self, mode):
        self.selected_function = mode
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
    # 2) page_scan
# ===========================================================================
    #버튼 연결
    def _connect_page_scan(self):
        self.btn_back2.clicked.connect(lambda: self.show_page('main'))

    #page_scan들어갔을 때 하는 일
    def _enter_scan(self):
        print(self.selected_function)
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
        info = data.split(',')
        self.scanned_data = info
        print(self.scanned_data)
        # 탑승권 번호 마지막 4자리 correct_password로 저장.
        last4 = str(info[0])[-4:]
        self.correct_password = f"#{last4}*"
        print(self.correct_password)
        # 얻을 거 다 얻으면 카메라 끄고 다음 페이지로
        if data:
            self.camera.stop()                 # 카메라 멈추고
            self.show_page('check_info')       # 다음 페이지로

# ===========================================================================
    # 3) page_check_info
# ===========================================================================
    #버튼 연결
    def _connect_page_check_info(self):
        # 확인 버튼: loading으로 이동
        self.btn_correct3.clicked.connect(lambda: self.show_page('loading'))
        # 재스캔 또는 뒤로 버튼: 스캔 데이터 리셋 후 scan으로 이동
        self.btn_retry3.clicked.connect(self._on_retry3)
        self.btn_back3.clicked.connect(self._on_retry3)

    #qr 정보 테이블위젯에 채우기
    def _enter_check_info(self):
        print(self.selected_function)
        # 스캔된 데이터로 테이블 채우기
        for i, val in enumerate(self.scanned_data):
            self.tableWidget3.setItem(i, 1, QTableWidgetItem(str(val)))

    #다시 스캔해야 할 경우
    def _on_retry3(self):
        self.scanned_data = []
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
        
# ===========================================================================
    # 4) page_loading
# ===========================================================================
    #버튼 연결
    def _connect_page_loading(self):
        self.btn_end_loading4.clicked.connect(self._route_from_loading)
        self.btn_deny_load4.clicked.connect(self._route_from_loading)
        self.btn_back4.clicked.connect(lambda: self.show_page('check_info'))

    #page_loading 들어갔을 때 하는 일
    def _enter_loading(self):
        self.load_weight=10000
        self.load_timer=QTimer(self)
        self.load_timer.timeout.connect(self._update_loading)
        self.load_timer.start(1000)

    #page_loading에서 나갈 때 하는 일
    def _exit_loading(self):
        self.load_timer.stop()

    #weight_bar 변화 시각화
    def _update_loading(self):
        self.load_weight += 500
        self.weight_bar_main4.setValue(self.load_weight)
        self.weight_bar_main4.setFormat(f"{self.load_weight/1000:.1f}kg")
        if self.load_weight >= 15000:
            self.load_timer.stop()

    #다음 페이지로 넘어갈 때 조건 분기
    def _route_from_loading(self):
        if self.selected_function == 'following_mode' :
            self.show_page('following')
        else:
            self.show_page('select_place')

# ===========================================================================
    # 5) page_following
# ===========================================================================
    #버튼 연결
    def _connect_page_following(self):
        self.btn_back5.clicked.connect(lambda: self.show_page('loading'))
        self.btn_end_following5.clicked.connect(lambda: self.show_page('bye'))
        self.btn_pause5.clicked.connect(self._toggle_following_pause)

    #page_following 들어갔을 때 하는 일
    def _enter_following(self):
        print(self.selected_function)
        self.follow_idx=0
        self.follow_timer=QTimer(self)
        self.follow_timer.timeout.connect(self._update_follow_text)
        self.follow_timer.start(2000)
        self._schedule_transition('bye', 10000)

    #page_following 나올 때 하는 일
    def _exit_following(self):
        if self._transition_timer.isActive():
            self._transition_timer.stop()
        self.follow_timer.stop()

    #following 할 때 알림텍스트 띄우기
    def _update_follow_text(self):
        msgs=["동행 중이에요 :)","너무 빨라요, 조금만 기다려줘요"]
        self.label_following_text5.setText(msgs[self.follow_idx])
        self.follow_idx=(self.follow_idx+1)%2

        #btn_pause 텍스트 변경
    def _toggle_following_pause(self):
        self.is_paused_f = not self.is_paused_f
        if self.is_paused_f:
            self.btn_pause5.setText("주행시작")
            self.show_page("waiting")
        else:
            self.btn_pause5.setText("일시정지")

# ===========================================================================
    # 6) page_select_place
# ===========================================================================
    #버튼 연결
    def _connect_page_select_place(self):
        for btn in self.findChildren(QPushButton):
            if btn.objectName().startswith('btn_place_'):
                btn.clicked.connect(lambda _,b=btn: self._toggle_destination(b))
        self.btn_back6.clicked.connect(self._go_back_page)
        self.btn_start6.clicked.connect(lambda: self.show_page('check_goal'))

    #page_select_place 들어갔을 때 하는 일
    def _enter_select_place(self):
        print(self.selected_function)

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
        print(self.selected_keys)
        print(self.destinations)
        print(self.arrived_place)

# ===========================================================================
    # 7) page_check_goal
# ===========================================================================
    #버튼 연결
    def _connect_page_check_goal(self):
        self.btn_start7.clicked.connect(self._start_guide)
        self.btn_change_place7.clicked.connect(lambda: self.show_page('select_place'))
        self.btn_back7.setEnabled(False)

    #들어갈 때 하는 일
    def _enter_check_goal(self):
        print(self.selected_function)
        print(self.selected_keys)
        print(self.destinations)
        print(self.arrived_place)

    #나올 때 하는 일
    def _start_guide(self):
        self.show_page('guiding')

# ===========================================================================
    # 8) page_guiding
# ===========================================================================
    #버튼 연결
    def _connect_page_guiding(self):
        self.btn_back8.setEnabled(False)
        self.btn_pause8.clicked.connect(self._toggle_guiding_pause)
        self.btn_change_place8.clicked.connect(lambda: self.show_page('select_place'))

    #들어갈 때 하는 일
    def _enter_guiding(self):
        # existing timers...
        self._schedule_transition('decision', 10000)
        print(self.selected_keys)
        print(self.destinations)
        print(self.arrived_place)

    #나올 때 하는 일
    def _exit_guiding(self):
        if self._transition_timer.isActive():
            self._transition_timer.stop()

    #btn_pause 텍스트 변경
    def _toggle_guiding_pause(self):
        self.is_paused_g = not self.is_paused_g
        if self.is_paused_g:
            self.btn_pause8.setText("이어서 안내시작")
            self.show_page("user_valid")
        else:
            self.btn_pause8.setText("일시정지")

# ===========================================================================
    # 9) page_decision_after_arrived
# ===========================================================================
    #버튼 연결
    def _connect_page_decision(self):
        self.btn_back9.setEnabled(False)
        self.btn_go_next_goal9.clicked.connect(lambda: self.show_page('guiding'))
        self.btn_change_goal9.clicked.connect(lambda: self.show_page('select_place'))
        self.btn_wait9.clicked.connect(lambda: self.show_page('waiting'))
        self.btn_bye9.clicked.connect(lambda: self.show_page('bye'))
    
    #page_decision_after_arrived 들어갈 때 하는 일
    def _enter_decision(self):
        print("I arrived")
        if not self.destinations:
            return
        arrived = self.destinations.pop(0)
        self.selected_keys.pop(0)
        self.arrived_place.append(arrived)
        self._refresh_place_buttons()

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


# ===========================================================================
    # 10) page_waiting
# ===========================================================================
    #버튼 연결
    def _connect_page_waiting(self):
        self.btn_back10.clicked.connect(lambda: self.show_page('user_valid'))
        self.btn_imback10.clicked.connect(lambda: self.show_page('user_valid'))

    #page_waiting 들어갈 때 하는 일
    def _enter_waiting(self):
        print(self.selected_function)

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
        print(self.selected_function)
        self.pushed_valid_btn = ''
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
                self.label_ps_text11.setText("인증 실패")
                self.pushed_valid_btn = ''

# ===========================================================================
    # 12) page_bye
# ===========================================================================

    def _connect_page_bye(self):
        self.btn_back12.setEnabled(False)

    def _enter_bye(self):
        print(self.selected_function)
        self.selected_function = None
        self.blink = False
        self.bye_timer = QTimer(self)
        self.bye_timer.timeout.connect(self._blink_bye)
        self.bye_timer.start(500)
        self._schedule_transition('charging', 5000)

    def _exit_bye(self):
        if self._transition_timer.isActive():
            self._transition_timer.stop()    
        self.bye_timer.stop()

    def _blink_bye(self):
        self.blink = not self.blink
        for lbl in [self.label_bye_icon12, self.label_bye_text12, self.label_bye_text12_2]:
            lbl.setVisible(self.blink)

# ===========================================================================
    # 13) page_charging
# ===========================================================================
    def _connect_page_charging(self):
        pass

    def _enter_charging(self):
        print(self.selected_function)
        self.charge_blink = False
        self.charge_timer = QTimer(self)
        self.charge_timer.timeout.connect(self._blink_charge)
        self.charge_timer.start(500)
        self.battery = 30
        self.batt_timer = QTimer(self); self.batt_timer.timeout.connect(self._update_battery); self.batt_timer.start(1000)

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

if __name__ == '__main__':
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
