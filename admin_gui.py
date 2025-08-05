import sys
import socket
import json
import struct
import threading
import yaml
import time
import random
import queue
import select
from math import cos, sin
from PyQt6 import QtWidgets, uic
from PyQt6.QtCore import pyqtSignal, QObject, QPointF, Qt, QThread, pyqtSlot, QTimer
from PyQt6.QtGui import QPixmap, QPen, QBrush, QColor, QTransform
from PyQt6.QtWidgets import (
    QGraphicsScene, QGraphicsPixmapItem, QGraphicsEllipseItem,
    QGraphicsLineItem, QListWidgetItem, QGraphicsView
)

HEADER_SIZE = 1
CMD_SIZE = 2

# client queue
send_queue = queue.Queue()

# server queue
login_result = queue.Queue()
robot_status = queue.Queue()
robot_detail = queue.Queue()
user_info = queue.Queue()

# ---------- Client thread ----------
def encode_message(cmd, json_obj):
    body = json.dumps(json_obj).encode('utf-8')
    length_bytes = struct.pack('>I', len(body))
    header = b'\x00'
    cmd_bytes = cmd.encode('utf-8')
    return header + length_bytes + cmd_bytes + body

class ClientThread(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(('192.168.0.145', 12348))

    def run(self):
        while True:
            # print(f"[ClientThread] Waiting for main service connection on {12348}...")
            if not send_queue.empty():
                data = send_queue.get()
                cmd = data[0]
                payload = data[1]
                msg = encode_message(cmd, payload)
                self.sock.sendall(msg)

# ---------- Server thread ----------
def decode_stream(cmd, data):
    try:
        if cmd == "LG":
            if data == b"\x01":
                payload = True
            elif data == b"\x00":
                payload = False
        else:
            payload = json.loads(data.decode('utf-8'))
            
        return payload
    
    except Exception as e:
        import traceback
        traceback.print_exc()
        print("JSON decode error: {e}")
        return
    
# ---------- Signal manager ----------
class SignalManager(QObject):
    login_result = pyqtSignal(bool)
    robot_status = pyqtSignal(list)
    robot_detail = pyqtSignal(dict)
    user_info = pyqtSignal(dict)

class ServerThread(QThread):  
    def __init__(self, signals=SignalManager, host="0.0.0.0", port=23456):
        super().__init__()
        self.signals = signals
        # print(type(self.signals))
        self.host = host
        self.port = port
        self.buffer = b""
        self.running = True
        self.conn = None

    def run(self):
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind((self.host, self.port))
        server_sock.listen()
        # server_sock.setblocking(False)  # 논블로킹 소켓으로 설정
        print(f"[ServerThread] Waiting for main service connection on {self.port}...")

        # conn = None
        while self.running:
            try:
                conn, addr = server_sock.accept()
                print(f"[ServerThread] Connected from {addr}")
                self.conn = conn  # store connection for future use
                          
                header = conn.recv(1)
                # print(header)
                if header != b'\x00':
                    # invalid header 처리
                    continue
                # print("header check")
                length_bytes = conn.recv(4)
                if len(length_bytes) < 4:
                    continue
                length = struct.unpack('>I', length_bytes)[0]
                # print("length check")
                cmd_bytes = conn.recv(2)
                if len(cmd_bytes) < 2:
                    continue
                cmd = cmd_bytes.decode('ascii')
                # print("cmd check")
                # 데이터 길이만큼 계속 받아야 함
                data_bytes = b''
                while len(data_bytes) < length:
                    chunk = conn.recv(length - len(data_bytes))
                    if not chunk:
                        # 연결 종료된 경우
                        raise ConnectionError("Connection closed by peer")
                    data_bytes += chunk
                
                data = decode_stream(cmd, data_bytes)
                print(f"cmd: {cmd}, data: {data}")

                self.handle_message(cmd, data)
                
            except Exception as e:
                import traceback
                traceback.print_exc
                print("[ServerThread] Error:", e)
                break

    def handle_message(self, cmd, payload):
        if cmd == "LG":
            self.signals.login_result.emit(payload)
            print(f"[DEBUG] emitting login_result with:", {cmd, type(payload)})
            # print("[DEBUG] SignalManager ID in ServerThread:", id(self.server_thread))  # ServerThread

        elif cmd == "RS":
            # if isinstance(payload["robots"], list):
            #     self.signals.robot_status.emit(payload["robots"], [])
            #     print(f"[DEBUG] emitting rbot_status with:", {cmd, type(payload)})
            # else:
            self.signals.robot_detail.emit(payload)
            print(f"[DEBUG] emitting robot_detail with:", {cmd, type(payload)})
                
        elif cmd == "UI":
            # self.signals.user_info.emit(payload["users"], [])
            self.signals.user_info.emit(payload)
            print(f"[DEBUG] emitting user info with:", {cmd, type(payload)})
            
        else:
            print(f"[DEBUG] emitting random with:", {cmd, type(payload)})

# ---------- Main GUI ----------
class AdminGUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("./admin_gui.ui", self)
        self.setWindowTitle("ARCS Admin GUI")
        # print("[DEBUG] SignalManager ID in GUI:", id(self.signals))  # AdminGUI
        
        # print(f"[DEBUG] login_result receivers: {self.signals.login_result.connect(self.on_login_result)}")
        # print(f"[DEBUG] login_result receivers count:", self.signals.login_result.receivers())

        # 시그널 연결
        self.signals = SignalManager()

        # 통신 시작
        self.client_thread = ClientThread()
        self.client_thread.start()
        
        # self.server_thread = ServerThread()
        self.server_thread = ServerThread(self.signals)
        self.server_thread.start()
        
        
        # self.signals = SignalManager(self)
        self.signals.login_result.connect(self.on_login_result)
        self.signals.robot_status.connect(self.update_robot_graphics)
        self.signals.robot_detail.connect(self.display_robot_detail)
        self.signals.user_info.connect(self.display_user_info)
        
        # self.signals.login_result.emit(True)
        # QTimer.singleShot(2000, self.server_thread.test_emit)
        
        # 버튼 연결
        self.loginButton.clicked.connect(self.start_login)
        self.queryButton.clicked.connect(self.request_user_info)
        self.robotListWidget.itemClicked.connect(self.on_robot_list_clicked)

        # 맵 초기화
        with open("./academy.yaml", 'r') as f:
            meta = yaml.safe_load(f)
            self.resolution = meta['resolution']
            self.origin = meta['origin'][:2]

        self.map_image = QPixmap("./academy.pgm")
        transform = QTransform().rotate(90)
        self.map_image = self.map_image.transformed(transform)
        self.scene = QGraphicsScene()
        self.map_item = QGraphicsPixmapItem(self.map_image)
        self.scene.addItem(self.map_item)
        self.mapView.setScene(self.scene)
        self.robot_items = {}

        # 로봇 리스트 초기화 (YAML 사용)
        self.load_robot_list("./robot_list.yaml")
        
        # 시간 위젯 설정
        self.hourCombo.addItem("")
        for h in range(24):
            self.hourCombo.addItem(f"{h:02d}")
        self.minuteSpin.setRange(0, 59)
        self.secondSpin.setRange(0, 59)
        

    def load_robot_list(self, path):
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
            robot_ids = data.get("robots", [])

        self.robotListWidget.clear()

        for robot_id in robot_ids:
            item = QListWidgetItem(robot_id)
            item.setForeground(QBrush(QColor("gray")))  # 초기 상태는 알 수 없음
            self.robotListWidget.addItem(item)

    def start_login(self):
        user_id = self.loginIdEdit.text()
        password = self.loginPwEdit.text()
        login_data = {"id": user_id, "password": password}
        
        data = ("LG", login_data)
        send_queue.put(data)
        # self.client_thread.run()
        
    # @pyqtSlot(bool)
    # def on_login_result(self, success):
    #     print("login checked")
    #     if success:
    #         self.tabWidget.setEnabled(True)
    #     else:
    #         QtWidgets.QMessageBox.warning(self, "Login Failed", "ID or password is incorrect.")
    @pyqtSlot(bool)
    def on_login_result(self, success):
        try:
            print("[DEBUG] on_login_result triggered with:", success)
            if success:
                self.tabWidget.setEnabled(True)
            else:
                QtWidgets.QMessageBox.warning(self, "Login Failed", "ID or password is incorrect.")
        except Exception as e:
            import traceback
            traceback.print_exc()
            print(f"Exception in on_login_result: {e}")

    def extract_robot_number(self, robot_str):
        if robot_str.startswith("ARCS_"):
            try:
                return int(robot_str.split("_")[1])
            except:
                return 0
        return 0
    
    def world_to_scene(self, x, y):
        # 맵이 90도 회전되었으므로 좌표 변환도 적용
        map_y = (x - self.origin[0]) / self.resolution
        map_x = (y - self.origin[1]) / self.resolution
        map_x = self.map_image.width() - map_x
        return QPointF(map_x, map_y)
    
    def update_robot_graphics(self, robot_list):
        for r_id, (dot, line) in self.robot_items.items():
            self.scene.removeItem(dot)
            self.scene.removeItem(line)
        self.robot_items.clear()
        for robot in robot_list:
            robot_id = robot['robot_id']
            x, y, theta = robot['x'], robot['y'], robot.get('theta', 0)
            pos = self.world_to_scene(x, y)
            dot = QGraphicsEllipseItem(pos.x() - 5, pos.y() - 5, 10, 10)
            dot.setBrush(QBrush(QColor("red")))
            dot.setToolTip(robot_id)
            dx, dy = 20 * cos(theta), -20 * sin(theta)
            line = QGraphicsLineItem(pos.x(), pos.y(), pos.x() + dx, pos.y() + dy)
            line.setPen(QPen(QColor("blue"), 2))
            self.scene.addItem(dot)
            self.scene.addItem(line)
            self.robot_items[robot_id] = (dot, line)
            dot.mousePressEvent = lambda e, rid=robot_id: self.on_robot_click(rid)

    def on_robot_click(self, robot_id):
        items = self.robotListWidget.findItems(robot_id, Qt.MatchFlag.MatchExactly)

        if items:
            self.robotListWidget.setCurrentItem(items[0])

        self.request_robot_detail({"robot_id": robot_id})

    def on_robot_list_clicked(self, item):
        robot_id = item.text()
        self.request_robot_detail({"robot_id": robot_id})
        
    def request_robot_detail(self, data):
        print("RS requested")
        data = ("RS", data)
        send_queue.put(data)
        # self.client_thread.run()

    def request_user_info(self):
        # 입력값 받아오기
        robot_id = self.robotIdEdit.text().strip()
        name = self.userNameEdit.text().strip()
        ticket = self.ticketEdit.text().strip()

        # 시간 조합
        hour = self.hourCombo.currentText()
        minute = self.minuteSpin.value()
        second = self.secondSpin.value()
        if hour == "":  # 시간 선택 안 했으면 전체 무시
            updated_time = None
        else:
            updated_time = f"{int(hour):02d}:{int(minute):02d}:{int(second):02d}"  # TIME 형식

        # 조건 딕셔너리 구성
        condition = {
            "robot_id": robot_id if robot_id else None,
            "name": name if name else None,
            "ticket_number": ticket if ticket else None,
            "updated_time": updated_time
        }

        # 디버그 출력용 WHERE 절 생성
        if all(v is None for v in condition.values()):
            print("[MySQL] SELECT * FROM USER")
        else:
            cond_list = []
            for k, v in condition.items():
                if v is not None:
                    if k == "updated_time":
                        cond_list.append(f"TIME(updated_time) = '{v}'")
                    else:
                        cond_list.append(f"{k} LIKE '%{v}%'")
            cond_str = " AND ".join(cond_list)
            print(f"[MySQL] WHERE {cond_str}")

        # 전송
        data = ("UI", condition)
        send_queue.put(data)

        # self.client_thread.run()
    @pyqtSlot(dict)
    def display_user_info(self, data):
        print("UI data displayed")
        if not data:
            self.queryResultText.setPlainText("찾는 사용자가 없습니다.")
            return
        
        # output = ""

        # if isinstance(data, dict) and "users" in data:
        #     user_list = data["users"]
        # elif isinstance(data, list):
        #     user_list = data
        # else:
        #     user_list = [data]
        # for user in user_list:
        #     output += "\n".join(f"{k}: {v}" for k, v in user.items())
        #     output += "\n---\n"
        output = data.get("key")
        self.queryResultText.setPlainText(output)
        
    @pyqtSlot(dict)
    def display_robot_detail(self, data):
        print("RS data displayed")
        if not data:
            self.robotStateLabel.setText("로봇 정보를 받아오지 못했습니다.")
            return
        
        # rid = data.get("robot_id", "")
        # x = data.get("x", 0)
        # y = data.get("y", 0)
        # status = data.get("status", "N/A")
        # battery = data.get("battery", "N/A")
        # task = data.get("task", "N/A")
        # user = data.get("user", "N/A")
        # self.robotStateLabel.setText(
        #     f"""[로봇 정보]
        #     ID: {rid}
        #     상태: {status}
        #     좌표: ({x:.2f}, {y:.2f})
        #     배터리: {battery}
        #     작업: {task}
        #     사용자: {user}
        #     """
        # )
        x = data.get("key")
        self.robotStateLabel.setText(x)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = AdminGUI()
    window.show()
    sys.exit(app.exec())
