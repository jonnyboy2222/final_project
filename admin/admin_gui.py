import sys
import socket
import json
import struct
import threading
import yaml
from math import cos, sin
from PyQt6 import QtWidgets, uic
from PyQt6.QtCore import pyqtSignal, QObject, QPointF, Qt
from PyQt6.QtGui import QPixmap, QPainter, QPen, QColor, QMouseEvent, QTransform

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
    total = HEADER_SIZE + CMD_SIZE + length
    if len(data) < total:
        return None
    cmd = data[HEADER_SIZE:HEADER_SIZE+CMD_SIZE].decode('utf-8')
    json_data = data[HEADER_SIZE+CMD_SIZE:total].decode('utf-8')
    payload = json.loads(json_data)
    return cmd, payload, data[total:]

# ---------- Signal manager ----------
class SignalManager(QObject):
    login_result = pyqtSignal(bool)
    robot_status = pyqtSignal(list)
    user_info = pyqtSignal(list)

# ---------- Client thread (to main service) ----------
class ClientThread(threading.Thread):
    def __init__(self, signals):
        super().__init__(daemon=True)
        self.signals = signals
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(('127.0.0.1', 9000))  # Connect to main service
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
                    cmd, payload, remaining = result
                    self.buffer = remaining
                    self.handle_message(cmd, payload)
            except Exception as e:
                print("[ClientThread] Error:", e)

    def handle_message(self, cmd, payload):
        if cmd == "LR":
            self.signals.login_result.emit(payload.get("success", False))
        elif cmd == "RS":
            self.signals.robot_status.emit(payload.get("robots", []))
        elif cmd == "UI":
            self.signals.user_info.emit(payload.get("users", []))

    def send(self, cmd, obj):
        message = encode_message(cmd, obj)
        self.sock.sendall(message)

    def request_user_info(self):
        self.send("UI", {})

# ---------- Server thread (receive from main service) ----------
class GUIReceiverThread(threading.Thread):
    def __init__(self, signals):
        super().__init__(daemon=True)
        self.signals = signals
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.bind(('0.0.0.0', 9001))  # Listen on port 9001
        self.server_sock.listen(1)

    def run(self):
        print("[GUIReceiver] Waiting for connection on port 9001...")
        conn, _ = self.server_sock.accept()
        print("[GUIReceiver] Connection established.")
        buffer = b""
        while True:
            try:
                data = conn.recv(4096)
                if not data:
                    continue
                buffer += data
                while True:
                    result = decode_stream(buffer)
                    if not result:
                        break
                    cmd, payload, remaining = result
                    buffer = remaining
                    if cmd == "RS":
                        self.signals.robot_status.emit(payload.get("robots", []))

            except Exception as e:
                print("[GUIReceiver] Error:", e)
                break

# ---------- Login wrapper thread ----------
class ServerThread(threading.Thread):
    def __init__(self, user_id, password, client_thread):
        super().__init__(daemon=True)
        self.user_id = user_id
        self.password = password
        self.client_thread = client_thread

    def run(self):
        login_data = {
            "id": self.user_id,
            "password": self.password
        }
        self.client_thread.send("LG", login_data)

# ---------- Main GUI ----------
class AdminGUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("admin_gui.ui", self)
        self.signals = SignalManager()
        self.signals.login_result.connect(self.on_login_result)
        self.signals.robot_status.connect(self.update_robot_status)
        self.signals.user_info.connect(self.display_user_info)
        self.loginButton.clicked.connect(self.start_server_thread)
        self.queryButton.clicked.connect(self.request_user_info)
        self.client_thread = ClientThread(self.signals)
        self.client_thread.start()
        self.receiver_thread = GUIReceiverThread(self.signals)
        self.receiver_thread.start()
        pixmap = QPixmap("/home/lee/final_project/admin/academy.pgm").scaled(600, 400)
        self.map_image = pixmap.transformed(QTransform().rotate(90))
        self.mapLabel.setPixmap(self.map_image)
        self.mapLabel.mousePressEvent = self.handle_map_click

        with open("/home/lee/final_project/admin/academy.yaml", 'r') as f:
            meta = yaml.safe_load(f)
            self.resolution = meta['resolution']
            self.origin = meta['origin'][:2]
            self.map_width = self.map_image.width()
            self.map_height = self.map_image.height()
        self.robot_positions = {}  # robot_id -> (map_x, map_y)
        self.selected_robot_id = None

    def start_server_thread(self):
        user_id = self.loginIdEdit.text()
        password = self.loginPwEdit.text()
        server_thread = ServerThread(user_id, password, self.client_thread)
        server_thread.start()

    def on_login_result(self, success):
        if success:
            self.tabWidget.setEnabled(True)
        else:
            QtWidgets.QMessageBox.warning(self, "Login Failed", "ID or password is incorrect.")

    def update_robot_status(self, robot_list):
        self.robot_positions.clear()
        pixmap = self.map_image.copy()
        painter = QPainter(pixmap)
        color_list = ["red", "blue", "green", "orange", "purple"]
        
        for i, robot in enumerate(robot_list):
            x, y, theta = robot['x'], robot['y'], robot.get('theta', 0)
            robot_id = robot['robot_id']
            map_x = int((x - self.origin[0]) / self.resolution)
            map_y = self.map_height - int((y - self.origin[1]) / self.resolution)
            self.robot_positions[robot_id] = (map_x, map_y)
            pen = QPen(QColor(color_list[i % len(color_list)]))
            pen.setWidth(4)
            painter.setPen(pen)
            # Draw position and heading
            painter.drawEllipse(QPointF(map_x, map_y), 6, 6)
            dx = 20 * cos(theta)
            dy = -20 * sin(theta)
            painter.drawLine(map_x, map_y, map_x + dx, map_y + dy)
            # Draw ID label
            painter.setPen(QColor("black"))
            painter.drawText(map_x + 8, map_y - 8, robot_id)
        painter.end()
        self.mapLabel.setPixmap(pixmap)

    def handle_map_click(self, event: QMouseEvent):
        clicked_x = event.position().x()
        clicked_y = event.position().y()

        for robot_id, (map_x, map_y) in self.robot_positions.items():
            if abs(clicked_x - map_x) <= 10 and abs(clicked_y - map_y) <= 10:
                self.selected_robot_id = robot_id
                QtWidgets.QMessageBox.information(self, "Robot Selected", f"You selected robot: {robot_id}")
                break

    def request_user_info(self):
        self.client_thread.request_user_info()

    def display_user_info(self, user_list):
        output = ""
        for user in user_list:
            output += "\n".join([f"{k}: {v}" for k, v in user.items()]) + "\n---\n"
        self.queryResultText.setPlainText(output)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = AdminGUI()
    window.show()
    sys.exit(app.exec())