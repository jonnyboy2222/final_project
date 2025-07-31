import sys
import socket
import json
import struct
import threading
import yaml
import time
from math import cos, sin
from PyQt6 import QtWidgets, uic
from PyQt6.QtCore import pyqtSignal, QObject, QPointF, Qt
from PyQt6.QtGui import QPixmap, QPen, QBrush, QColor, QTransform
from PyQt6.QtWidgets import (
    QGraphicsScene, QGraphicsPixmapItem, QGraphicsEllipseItem,
    QGraphicsLineItem, QListWidgetItem
)

# ---------- Protocol utils ----------
HEADER_SIZE = 4
CMD_SIZE = 2

def encode_message(cmd, json_obj):
    body = json.dumps(json_obj).encode('utf-8')
    length = len(body)
    header = struct.pack('>I', length)
    cmd_bytes = cmd.encode('utf-8')
    return header + cmd_bytes + body

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
        payload = bool(raw_data[0])

    else:
        payload = json.loads(raw_data.decode('utf-8'))

    return cmd, payload, data[total:]

# ---------- Signal manager ----------
class SignalManager(QObject):
    login_result = pyqtSignal(bool)
    robot_status = pyqtSignal(list)
    robot_detail = pyqtSignal(dict)
    user_info = pyqtSignal(list)

# ---------- Client thread ----------
class ClientThread(threading.Thread):
    def __init__(self, signals):
        super().__init__(daemon=True)
        self.signals = signals
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(('127.0.0.1', 9000))
        self.buffer = b""
        self.running = True

    def run(self):
        while self.running:
            try:
                data = self.sock.recv(4096)

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
                print("[ClientThread] Error:", e)

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

    def send(self, cmd, obj):
        message = encode_message(cmd, obj)
        self.sock.sendall(message)

    def request_user_info(self, keyword=None):
        if keyword:
            self.send("UI", {"search": keyword})
        else:
            self.send("UI", {})

    def request_robot_detail(self, robot_id_number):
        self.send("RS", {"robot_id": robot_id_number})

# ---------- Main GUI ----------
class AdminGUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("/home/lee/final_project/admin/admin_gui.ui", self)
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
        with open("/home/lee/final_project/admin/academy.yaml", 'r') as f:
            meta = yaml.safe_load(f)
            self.resolution = meta['resolution']
            self.origin = meta['origin'][:2]

        self.map_image = QPixmap("/home/lee/final_project/admin/academy.pgm")
        transform = QTransform().rotate(90)
        self.map_image = self.map_image.transformed(transform)
        self.scene = QGraphicsScene()
        self.map_item = QGraphicsPixmapItem(self.map_image)
        self.scene.addItem(self.map_item)
        self.mapView.setScene(self.scene)
        self.robot_items = {}

        # 로봇 리스트 초기화
        self.load_robot_list("/home/lee/final_project/admin/robot_list.yaml")

    def load_robot_list(self, path):
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
            robot_ids = data.get("robots", [])

        self.robotListWidget.clear()

        for robot_id in robot_ids:
            item = QListWidgetItem(robot_id)
            item.setForeground(QBrush(QColor("gray")))
            self.robotListWidget.addItem(item)

    def extract_robot_number(self, robot_id):
        if robot_id.startswith("ARCS_"):
            try:
                return int(robot_id.split("_")[1])
            except ValueError:
                return 0
        return 0
    
    def start_login(self):
        user_id = self.loginIdEdit.text()
        password = self.loginPwEdit.text()
        login_data = {"id": user_id, "password": password}
        self.client_thread.send("LG", login_data)

    def on_login_result(self, success):
        if success:
            self.tabWidget.setEnabled(True)
        else:
            QtWidgets.QMessageBox.warning(self, "Login Failed", "ID or password is incorrect.")

    def world_to_scene(self, x, y):
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
        robot_num = self.extract_robot_number(robot_id)
        self.client_thread.request_robot_detail(robot_num)

    def on_robot_list_clicked(self, item):
        robot_id = item.text()
        robot_num = self.extract_robot_number(robot_id)
        self.client_thread.request_robot_detail(robot_num)

    def request_user_info(self):
        keyword = self.queryTextEdit.toPlainText().strip()
        if keyword:
            self.client_thread.request_user_info(keyword)
        else:
            self.client_thread.request_user_info()

    def display_user_info(self, user_list):
        if not user_list:
            self.queryResultText.setPlainText("찾는 사용자가 없습니다.")
            return
        
        output = ""

        for user in user_list:
            output += "\n".join([f"{k}: {v}" for k, v in user.items()]) + "\n---\n"
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