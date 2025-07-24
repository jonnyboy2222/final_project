import sys
import cv2
import datetime
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QPushButton, QTableWidgetItem
from PyQt5.QtCore import QTimer, QThread, pyqtSignal
from PyQt5.uic import loadUi
import re
from PyQt5.QtGui import QImage, QPixmap


class CameraThread(QThread):
    frame_received = pyqtSignal(QImage)

    def __init__(self, index=0):
        super().__init__()
        self.index = index
        self.running = False

    def run(self):
        cap = cv2.VideoCapture(self.index)
        self.running = True
        while self.running:
            ret, frame = cap.read()
            if ret:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb.shape
                img = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()
                self.frame_received.emit(img)
        cap.release()
    def stop(self):
        self.running = False
        self.quit()
        self.wait()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        loadUi("client.ui", self)

        # 상태 변수
        self.selected_function = None        #following모드인지 leading 모드인지
        self.correct_password = "#3631*"
        self.pushed_valid_btn = ""

        # 공통: 실시간 상단바 시계
        self.dt_timer = QTimer(self)
        self.dt_timer.timeout.connect(self.update_datetime)
        self.dt_timer.start(1000)

        # 카메라 스레드
        self.camera = CameraThread(0)
        self.camera.frame_received.connect(self.on_camera_frame)

        # 페이지 매핑
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

        # 공통: 홈 버튼
        for btn in self.findChildren(QPushButton):
            if btn.objectName().startswith("btn_home"):
                btn.clicked.connect(self.on_home_clicked)

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

        # 초기 페이지
        self.current_page = None
        self.before_page = None
        self.show_page('main')

    #상단바 날짜 및 시간 업데이트 함수
    def update_datetime(self):
        now = datetime.datetime.now()
        weekdays = ['월','화','수','목','금','토','일']
        wd = weekdays[now.weekday()]
        txt = now.strftime(f"%Y.%m.%d ({wd}) %H:%M")
        for lbl in self.findChildren(QLabel):
            if lbl.objectName().startswith("label_datetime"):
                lbl.setText(txt)

    #홈 버튼 누르면            
    def on_home_clicked(self):
        self.selected_function = None
        self.show_page('main')

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

    # 1) page_main
    def _connect_page_main(self):
        self.btn_kor1.clicked.connect(self._to_korean)
        self.btn_eng1.clicked.connect(self._to_english)
        self.btn_follow1.clicked.connect(lambda: self._select_and_scan('following_mode'))
        self.btn_guide1.clicked.connect(lambda: self._select_and_scan('leading_mode'))

    def _to_korean(self):
        if getattr(self, 'language_mode', 'kor') != 'kor':
            self.language_mode = 'kor'
            # TODO: 전체 UI를 한글로 변환

    def _to_english(self):
        if getattr(self, 'language_mode', 'kor') == 'kor':
            self.language_mode = 'eng'
            # TODO: 전체 UI를 영어로 변환

    def _enter_main(self):
        self.selected_function = None
        print(self.selected_function)

    def _select_and_scan(self, mode):
        self.selected_function = mode
        self.show_page('scan')

    # 2) page_scan
    def _connect_page_scan(self):
        self.btn_back2.clicked.connect(lambda: self.show_page('main'))

    def _enter_scan(self):
        self.camera.start()
        print(self.selected_function)
        self._schedule_transition('check_info', 3000)

    def _exit_scan(self):
        if self._transition_timer.isActive():
            self._transition_timer.stop()
        self.camera.stop()

    # 3) page_check_info
    def _connect_page_check_info(self):
        self.btn_correct3.clicked.connect(lambda: self.show_page('loading'))
        self.btn_retry3.clicked.connect(lambda: self.show_page('scan'))
        self.btn_back3.clicked.connect(lambda: self.show_page('scan'))

    def _enter_check_info(self):
        print(self.selected_function)
        data = ['이동연','미국','2025.08.01 (금) 15:34','gate3']
        for i, txt in enumerate(data):
            self.tableWidget3.setItem(i, 1, QTableWidgetItem(txt))

    # 4) page_loading
    def _connect_page_loading(self):
        self.btn_end_loading4.clicked.connect(self._route_from_loading)
        self.btn_deny_load4.clicked.connect(self._route_from_loading)
        self.btn_back4.clicked.connect(lambda: self.show_page('check_info'))

    def _enter_loading(self):
        print(self.selected_function)
        self.load_value = 10000
        self.load_timer = QTimer(self)
        self.load_timer.timeout.connect(self._update_loading)
        self.load_timer.start(1000)

    def _exit_loading(self):
        self.load_timer.stop()

    def _update_loading(self):
        self.load_value += 500
        self.weight_bar_main4.setValue(self.load_value)
        self.weight_bar_main4.setFormat(f"{self.load_value/1000:.1f}kg")
        if self.load_value >= 15000:
            self.load_timer.stop()

    def _route_from_loading(self):
        if self.selected_function == 'following_mode' :
            self.show_page('following')
        else:
            self.show_page('select_place')

    # 5) page_following
    def _connect_page_following(self):
        self.btn_back5.clicked.connect(lambda: self.show_page('loading'))

    def _enter_following(self):
        print(self.selected_function)
        self.camera.start()
        self.follow_idx = 0
        self.follow_timer = QTimer(self)
        self.follow_timer.timeout.connect(self._update_following_text)
        self.follow_timer.start(2000)
        
        self._schedule_transition('bye', 10000)

    def _exit_following(self):
        if self._transition_timer.isActive():
            self._transition_timer.stop()
        self.follow_timer.stop()
        self.camera.stop()

    def _update_following_text(self):
        msgs = ["동행 중이에요 :)", "너무 빨라요, 조금만 기다려줘요"]
        self.label_following_text5.setText(msgs[self.follow_idx])
        self.follow_idx = (self.follow_idx + 1) % len(msgs)

    # 6) page_select_place
    def _connect_page_select_place(self):
        self.btn_start6.clicked.connect(lambda: self.show_page('check_goal'))
        self.btn_back6.clicked.connect(lambda: self.show_page('loading'))

    def _enter_select_place(self):
        print(self.selected_function)
        # 초기 예상 소요시간을 "분 초" 형식에서 파싱
        text = self.label_time6.text()
        nums = re.findall(r"(\d+)", text)
        if len(nums) == 2:
            minutes, seconds = map(int, nums)
            self.sp_remain = minutes * 60 + seconds
        elif len(nums) == 1:
            self.sp_remain = int(nums[0])
        else:
            self.sp_remain = 0

        # 타이머 시작
        self.sp_timer = QTimer(self)
        self.sp_timer.timeout.connect(self._update_select_time)
        self.sp_timer.start(1000)


    def _exit_select_place(self):
        self.sp_timer.stop()

    def _update_select_time(self):
        # 1초씩 감소시키며 "분 초" 형식으로 업데이트
        if self.sp_remain > 0:
            self.sp_remain -= 1
        minutes = self.sp_remain // 60
        seconds = self.sp_remain % 60
        self.label_time6.setText(f"{minutes} 분 {seconds}초")


    # 7) page_check_goal
    def _connect_page_check_goal(self):
        self.btn_start7.clicked.connect(lambda: self.show_page('guiding'))
        self.btn_change_place7.clicked.connect(lambda: self.show_page('select_place'))
        self.btn_back7.clicked.connect(lambda: self.show_page('select_place'))

    def _enter_check_goal(self):
        print(self.selected_function)

    
    # 8) page_guiding
    def _connect_page_guiding(self):
        self.btn_back8.clicked.connect(lambda: self.show_page('check_goal'))
        self.btn_pause8.clicked.connect(self._toggle_guiding_pause)
        self.btn_change_place8.clicked.connect(lambda: self.show_page('select_place'))

    def _enter_guiding(self):
        print(self.selected_function)
        self.g_decrease = True
        # 시간 값 추출
        text = self.label_time8.text()
        nums = re.findall(r"(\d+)", text)
        if len(nums) == 2:
            minutes, seconds = map(int, nums)
            self.remained_time = minutes * 60 + seconds
        elif len(nums) == 1:
            self.remained_time = int(nums[0])
        else:
            self.remained_time = 0
  
        # 거리 값 추출
        dist_txt = self.label_distance8.text()
        nums2 = re.findall(r"(\d+\.?\d*)", dist_txt)
        self.remained_dist = float(nums2[0]) if nums2 else 0.0

        self.g_msg_idx = 0
        # 기존 타이머 설정(변경 없음)
        self.g_t1 = QTimer(self)
        self.g_t1.timeout.connect(self._update_guiding_time)
        self.g_t1.start(1000)

        self.g_t2 = QTimer(self)
        self.g_t2.timeout.connect(self._update_guiding_dist)
        self.g_t2.start(1000)

        self.g_t3 = QTimer(self)
        self.g_t3.timeout.connect(self._update_guiding_text)
        self.g_t3.start(1000)

        self._schedule_transition('decision', 10000)

    def _exit_guiding(self):
        if self._transition_timer.isActive():
            self._transition_timer.stop()
        for t in (self.g_t1, self.g_t2, self.g_t3): t.stop()

    def _update_guiding_time(self):
        if self.g_decrease and self.remained_time > 0:
            self.remained_time -= 1
        minutes = self.remained_time // 60
        seconds = self.remained_time % 60
        self.label_time8.setText(f"{minutes} 분 {seconds}초")

    def _update_guiding_dist(self):
        if self.g_decrease and self.remained_dist > 0:
            self.remained_dist -= 0.01
        self.label_distance8.setText(f"{self.remained_dist:.2f}m")

    def _update_guiding_text(self):
        msgs = ["목적지로 길 안내중이에요 :)", "앞에 장애물이 있으니 조심하세요"]
        idx = getattr(self, 'g_msg_idx', 0)
        self.label_check_place_text8.setText(msgs[idx])
        self.g_msg_idx = (idx + 1) % len(msgs)

    def _toggle_guiding_pause(self):
        self.g_decrease = not self.g_decrease

    # 9) page_decision_after_arrived
    def _connect_page_decision(self):
        self.btn_back9.setEnabled(False)
        self.btn_go_next_goal9.clicked.connect(lambda: self.show_page('guiding'))
        self.btn_change_goal9.clicked.connect(lambda: self.show_page('select_place'))
        self.btn_wait9.clicked.connect(lambda: self.show_page('waiting'))
        self.btn_bye9.clicked.connect(lambda: self.show_page('bye'))
    
    def _enter_decision(self):
        print(self.selected_function)

    # 10) page_waiting
    def _connect_page_waiting(self):
        self.btn_back10.clicked.connect(lambda: self.show_page('user_valid'))
        self.btn_imback10.clicked.connect(lambda: self.show_page('user_valid'))

    def _enter_waiting(self):
        print(self.selected_function)
        self.camera.start()

    def _exit_waiting(self):
        self.camera.stop()

    # 11) page_user_valid
    def _connect_page_user_valid(self):
        # pg = self.page_user_valid
        # pg.btn_back11.clicked.connect(lambda: self.show_page('waiting'))
        self.btn_back11.clicked.connect(lambda: self.show_page('waiting'))

        # '*' → 'star', '#' → 'sharp' 매핑
        special = {'*': 'star', '#': 'sharp'}
        for key in ['1','2','3','4','5','6','7','8','9','0','*','#']:
            attr = special.get(key, key)               # '*'이면 'star', '#'이면 'sharp'
            btn = getattr(self, f'btn_{attr}')         # btn_star, btn_sharp
            btn.clicked.connect(lambda _, k=key: self._handle_user_input(k))

    def _enter_user_valid(self):
        print(self.selected_function)
        self.pushed_valid_btn = ''
        self.label_ps_text11.setText('비밀번호를 입력해주세요')

    def _handle_user_input(self, key):
        self.pushed_valid_btn += key
        if key == '*':
            if self.pushed_valid_btn == self.correct_password:
                self.show_page('decision')
            else:
                self.pushed_valid_btn = ''
                self.label_ps_text11.setText("인증 실패")

    # 12) page_bye
    def _connect_page_bye(self):
        self.btn_back12.setEnabled(False)

    def _enter_bye(self):
        print(self.selected_function)
        self.blink = False
        self.bye_timer = QTimer(self); self.bye_timer.timeout.connect(self._blink_bye); self.bye_timer.start(500)
        self._schedule_transition('charging', 5000)

    def _exit_bye(self):
        if self._transition_timer.isActive():
            self._transition_timer.stop()    
        self.bye_timer.stop()

    def _blink_bye(self):
        self.blink = not self.blink
        for lbl in [self.label_bye_icon12, self.label_bye_text12, self.label_bye_text12_2]:
            lbl.setVisible(self.blink)

    # 13) page_charging
    def _connect_page_charging(self):
        pass

    def _enter_charging(self):
        print(self.selected_function)
        self.charge_blink = False
        self.charge_timer = QTimer(self); self.charge_timer.timeout.connect(self._blink_charge); self.charge_timer.start(500)
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

    # 카메라 프레임 처리
    def on_camera_frame(self, img):
        if self.current_page == 'scan':
            self.label_camera_area2.setPixmap(QPixmap.fromImage(img))
        elif self.current_page == 'following':
            self.label_camera_area5.setPixmap(QPixmap.fromImage(img))
        elif self.current_page == 'waiting':
            self.label_camera_area10.setPixmap(QPixmap.fromImage(img))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
