import sys
import socket
import json
import struct
import threading
import yaml
import time
import random
from math import cos, sin
from PyQt6 import QtWidgets, uic
from PyQt6.QtCore import pyqtSignal, QObject, QPointF, Qt
from PyQt6.QtGui import QPixmap, QPen, QBrush, QColor, QTransform
from PyQt6.QtWidgets import (
    QGraphicsScene, QGraphicsPixmapItem, QGraphicsEllipseItem,
    QGraphicsLineItem, QListWidgetItem
)

HEADER_SIZE = 4
CMD_SIZE = 2

# ---------- Client thread ----------
def encode_message(cmd, json_obj):
    body = json.dumps(json_obj).encode('utf-8')
    length = len(body)
    header = struct.pack('>I', length)
    cmd_bytes = cmd.encode('utf-8')
    return header + cmd_bytes + body

class ClientThread(threading.Thread):
    def __init__(self, send_queue):
        super().__init__(daemon=True)
        self.send_queue = send_queue
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(('192.168.0.145', 12348))

    def run(self):
        while True:
            cmd, payload = self.send_queue.get()
            msg = encode_message(cmd, payload)
            self.sock.sendall(msg)

# ---------- Server thread ----------
def decode_stream(data):
    if len(data) < HEADER_SIZE + CMD_SIZE:
        return None
    length = struct.unpack('>I', data[:HEADER_SIZE])[0]
    total = HEADER_SIZE + length + CMD_SIZE
    if len(data) < total:
        return None
    cmd = data[HEADER_SIZE:HEADER_SIZE+CMD_SIZE].decode('utf-8')
    raw_data = data[HEADER_SIZE+CMD_SIZE:total]
    if cmd == "LG":
        if raw_data == "0x01":
            payload = 1
        elif raw_data == "0x00":
            payload = 0
    else:
        payload = json.loads(raw_data.decode('utf-8'))
    return cmd, payload, data[total:]

class ServerThread(threading.Thread):
    def __init__(self, signals, host="0.0.0.0", port=23456):
        super().__init__(daemon=True)
        self.signals = signals
        self.host = host
        self.port = port
        self.buffer = b""
        self.running = True

    def run(self):
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind((self.host, self.port))
        server_sock.listen(1)
        print(f"[ServerThread] Waiting for main service connection on {self.port}...")

        conn, addr = server_sock.accept()
        print(f"[ServerThread] Connected from {addr}")

        while self.running:
            try:
                data = conn.recv(4096)
                if not data:
                    continue
                self.buffer += data
                while True:
                    result = decode_stream(self.buffer)
                    if not result:
                        break
                    cmd, payload, self.buffer = result
                    self.handle_message(cmd, payload)
            except Exception as e:
                print("[ServerThread] Error:", e)
                break

    def handle_message(self, cmd, payload):
        if cmd == "LG":
            self.signals.login_result.emit(payload)
        elif cmd == "RS":
            if isinstance(payload.get("robots"), list):
                self.signals.robot_status.emit(payload.get("robots", []))
            else:
                self.signals.robot_detail.emit(payload)
        elif cmd == "UI":
            self.signals.user_info.emit(payload.get("users", []))

# ---------- Signal manager ----------
class SignalManager(QObject):
    login_result = pyqtSignal(bool)
    robot_status = pyqtSignal(list)
    robot_detail = pyqtSignal(dict)
    user_info = pyqtSignal(list)

# ---------- Main GUI ----------
class AdminGUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("./admin_gui.ui", self)
        self.setWindowTitle("ARCS Admin GUI")

        # 시그널 연결
        self.signals = SignalManager()
        self.signals.login_result.connect(self.on_login_result)
        self.signals.robot_status.connect(self.update_robot_graphics)
        self.signals.robot_detail.connect(self.display_robot_detail)
        self.signals.user_info.connect(self.display_user_info)

        # 버튼 연결
        self.loginButton.clicked.connect(self.start_login)
        self.queryButton.clicked.connect(self.request_user_info)
        self.robotListWidget.itemClicked.connect(self.on_robot_list_clicked)

        # 통신 시작
        self.client_thread = ClientThread(self.signals)
        self.client_thread.start()

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
        self.client_thread.send("LG", login_data)

    def on_login_result(self, success):
        if success == 1:
            self.tabWidget.setEnabled(True)
        else:
            QtWidgets.QMessageBox.warning(self, "Login Failed", "ID or password is incorrect.")

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

        self.client_thread.request_robot_detail({"robot_id": robot_id})

    def on_robot_list_clicked(self, item):

        robot_id = item.text()

        self.client_thread.request_robot_detail({"robot_id": robot_id})

    def request_user_info(self):
        # 입력값 받아오기
        robot_id = self.robotIdEdit.text().strip()
        name = self.nameEdit.text().strip()
        ticket = self.ticketEdit.text().strip()

        # 시간 조합
        hour = self.hourCombo.currentText()
        minute = self.minuteSpin.value()
        second = self.secondSpin.value()
        if hour == "":  # 시간 선택 안했으면 시간 전체 무시
            updated_time = None
        else:
            updated_time = f"{int(hour):02d}:{int(minute):02d}:{int(second):02d}"

        # 조건 딕셔너리 구성
        condition = {
            "robot_id": robot_id if robot_id else None,
            "name": name if name else None,
            "ticket_number": ticket if ticket else None,
            "updated_time": updated_time
        }

        # 디버그 출력용 WHERE
        if all(v is None for v in condition.values()):
            print("[MySQL] SELECT * FROM USER")
        else:
            cond_str = " AND ".join(f"{k} LIKE '%{v}%'" for k, v in condition.items() if v is not None)
            print(f"[MySQL] WHERE {cond_str}")

        # 전송
        self.client_thread.send("UI", condition)

    def display_user_info(self, data):
        if not data:
            self.queryResultText.setPlainText("찾는 사용자가 없습니다.")
            return
        
        output = ""

        if isinstance(data, dict) and "users" in data:
            user_list = data["users"]
        elif isinstance(data, list):
            user_list = data
        else:
            user_list = [data]
        for user in user_list:
            output += "\n".join(f"{k}: {v}" for k, v in user.items())
            output += "\n---\n"
        self.queryResultText.setPlainText(output)

    def display_robot_detail(self, data):
        if not data:
            self.robotStateLabel.setText("로봇 정보를 받아오지 못했습니다.")
            return
        
        rid = data.get("robot_id", "")
        x = data.get("x", 0)
        y = data.get("y", 0)
        status = data.get("status", "N/A")
        battery = data.get("battery", "N/A")
        task = data.get("task", "N/A")
        user = data.get("user", "N/A")
        self.robotStateLabel.setText(
            f"""[로봇 정보]
            ID: {rid}
            상태: {status}
            좌표: ({x:.2f}, {y:.2f})
            배터리: {battery}
            작업: {task}
            사용자: {user}
            """
        )

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = AdminGUI()
    window.show()
    sys.exit(app.exec())
