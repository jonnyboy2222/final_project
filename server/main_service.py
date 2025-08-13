import socket
import struct
import threading
import json
import time
from datetime import datetime
import queue
import multiprocessing
from multiprocessing import Process, Queue, Manager, Value

from db_connect import ARCSDatabaseHandler


# 서버 ip
tcp_ip = '0.0.0.0'

udp_ip = '0.0.0.0'

# Main Controller용 포트
udp_port = 54321

# Vision용 포트
vision_tcp_port = 12345

# Chat용 포트
llm_tcp_port = 12346

# User용 포트
user_tcp_port = 12347

# Admin용 포트
admin_tcp_port = 12348



admin_ip = '192.168.0.20'      # Admin PC IP
admin_port = 23456             # Admin TCP 포트

user_ip = '192.168.0.30'       # User PC IP
user_port = 23457              # User TCP 포트

main_udp_ip = '192.168.5.9'  # 메인 컨트롤러 IP
main_udp_port = 23458          # 메인 컨트롤러 포트

llm_tcp_ip = "192.168.0.43"    # Chat Service IP
llm_port = 23459               # Chat Service TCP 포트


    
# TCP 데이터 수신
def start_vision_tcp_server(tcp_ip, vision_tcp_port, running, vision_data_queue, distance_queue):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.bind((tcp_ip, vision_tcp_port))
        server.listen()
        print(f"[VISION SERVICE TCP] Listening on {tcp_ip}:{vision_tcp_port}")

        while running.value:
            conn, addr = server.accept()
            threading.Thread(target=vision_tcp_receiver, args=(conn, addr, vision_data_queue, distance_queue), daemon=True).start()

def vision_tcp_receiver(conn, addr, vision_data_queue, distance_queue):
    print(f"[VISION SERVICE] Connected from {addr}")
    try:
        while True:
            # 1) Read exactly 1 byte for the header
            header = conn.recv(1)
            
            if header != b'\x00':
                print(f"[VISION SERVICE] Invalid header: {header}")
                return
            # print(f"[VISION SERVICE] Header OK: {header.hex()}")

            
            # 2) Read exactly 4 bytes for the length
            length_bytes = conn.recv(4)
            if len(length_bytes) < 4:
                print("[VISION SERVICE] Connection closed before length received")
                return

            # Unpack as unsigned int, big-endian
            (length,) = struct.unpack('>I', length_bytes)
            # e.g. length = 1234

            # 3) Read exactly `length` bytes
            data = b''
            remaining = length
            while remaining > 0:
                packet = conn.recv(remaining)
                if not packet:
                    print(f"[VISION SERVICE] Connection closed with {remaining} bytes left to read")
                    return
                data += packet
                remaining -= len(packet)

            # 4) Decode and enqueue
            json_data = json.loads(data.decode('utf-8'))
            # print(f"[VISION SERVICE] Data: {json_data}")
            distance = json_data.get("distance")

            distance_queue.put(distance)

            vision_data_queue.put(data)

    except Exception as e:
        print(f"[VISION SERVICE ERROR] {e}")
    finally:
        conn.close()
        print(f"[VISION SERVICE] Connection closed from {addr}")


def start_llm_tcp_server(tcp_ip, llm_tcp_port, running, llm_data_queue, pos_queue):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.bind((tcp_ip, llm_tcp_port))
        server.listen()
        print(f"[CHAT SERVICE TCP] Listening on {tcp_ip}:{llm_tcp_port}")

        while running.value:
            conn, addr = server.accept()
            threading.Thread(target=llm_tcp_receiver, args=(conn, addr, llm_data_queue, pos_queue), daemon=True).start()


def llm_tcp_receiver(conn, addr, llm_data_queue, pos_queue):
    # print(f"[CHAT SERVICE] Connected from {addr}")
    try:
        # 위치 전송 스레드 시작
        pos_thread = threading.Thread(target=send_pos_loop, args=(pos_queue,), daemon=True)
        pos_thread.start()

        while True:
            # 1) Header
            header = conn.recv(1)
            if header != b'\x00':
                # print(f"[CHAT SERVICE] Invalid header: {header}")
                return
            # print(f"[CHAT SERVICE] Header OK: {header.hex()}")

            # 2) Length
            length_bytes = conn.recv(4)
            if len(length_bytes) < 4:
                print("[CHAT SERVICE] Connection closed before length received")
                return
            (length,) = struct.unpack('>I', length_bytes)

            # 3) Payload
            data = b''
            remaining = length
            while remaining > 0:
                packet = conn.recv(remaining)
                if not packet:
                    print(f"[CHAT SERVICE] Connection closed with {remaining} bytes left")
                    return
                data += packet
                remaining -= len(packet)

            # 4) Decode and enqueue
            json_data = data.decode('utf-8')
            print(f"[CHAT SERVICE] Data: {json_data}")
            llm_data_queue.put(data)

    except Exception as e:
        print(f"[CHAT SERVICE ERROR] {e}")
    finally:
        conn.close()
        # print(f"[CHAT SERVICE] Connection closed from {addr}")

def send_pos_loop(pos_queue):
    sock = None

    while True:
        try:
            # 연결이 안 돼있으면 연결 시도
            if sock is None:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((llm_tcp_ip, llm_port))
                # print("[CHAT SERVICE TCP] Connected to LLM (persistent)")

            # 위치 전송
            if not pos_queue.empty():
                current_pos = pos_queue.get()
                # print(f"[CHAT SERVICE] Current Position: {current_pos}")
                current_pos_bytes = json.dumps(current_pos).encode('utf-8')

                header = b'\x00'
                length_bytes = struct.pack('>I', len(current_pos_bytes))
                packet = header + length_bytes + current_pos_bytes
                sock.sendall(packet)
                print(f"[CHAT SERVICE TCP] Sent position: {packet}")
                time.sleep(2)

        except (BrokenPipeError, ConnectionResetError, OSError) as e:
            # print(f"[CHAT SERVICE TCP] Connection lost: {e}, retrying...")
            if sock:
                try:
                    sock.close()
                except:
                    pass
            sock = None  # 다음 루프에서 재연결
            time.sleep(1)


def start_user_tcp_server(tcp_ip, user_tcp_port, running, vision_data_queue, robot_id_queue, already_checked):
    # --- 추가: VD 주기 송신 On/Off ---
    vd_enabled = threading.Event()

    # --- 추가: 최신 비전 데이터 공유(스레드 간) ---
    vision_lock = threading.Lock()
    latest_vision = {"bytes": None}  # 최신 원본 바이트

    # 최신 비전 데이터 수집 스레드 (vision_data_queue -> latest_vision)
    threading.Thread(
        target=vision_latest_collector,
        args=(vision_data_queue, latest_vision, vision_lock, running),
        daemon=True
    ).start()

    # VD 주기 송신 스레드 (초당 1회)
    threading.Thread(
        target=vd_sender_loop,
        args=(vd_enabled, latest_vision, vision_lock, running),
        daemon=True
    ).start()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((tcp_ip, user_tcp_port))
        server.listen()
        print(f"[User TCP] Listening on {tcp_ip}:{user_tcp_port}")

        while running.value:
            print("[SERVER] 대기 중: accepting connection...")
            conn, addr = server.accept()
            print(f"[SERVER] 새 연결 수신: {addr}")
            threading.Thread(
                target=user_tcp_receiver,
                args=(conn, addr, latest_vision, vision_lock, robot_id_queue, already_checked, vd_enabled),
                daemon=True
            ).start()


def vision_latest_collector(vision_data_queue, latest_vision, vision_lock, running):
    while running.value:
        try:
            data = vision_data_queue.get(timeout=0.2)  # 블로킹으로 받아도 OK
            with vision_lock:
                latest_vision["bytes"] = data
        except queue.Empty:
            pass
        except Exception as e:
            print(f"[VISION LATEST COLLECTOR ERROR] {e}")
            time.sleep(0.5)

def vd_sender_loop(vd_enabled, latest_vision, vision_lock, running):
    while running.value:
        try:
            if vd_enabled.is_set():
                with vision_lock:
                    vd_payload = latest_vision["bytes"]
                if vd_payload is not None:
                    # VD는 des_coor_list가 없고, vision_data 바이트만 보냄
                    send_to_main_controller_udp("VD", None, vd_payload)
            time.sleep(1.0)  # 초당 1회
        except Exception as e:
            print(f"[VD LOOP ERROR] {e}")
            time.sleep(1.0)


def user_tcp_receiver(conn, addr, latest_vision, vision_lock, robot_id_queue, already_checked, vd_enabled):
    print(f"[USER] Connected from {addr}")
    try:
        # arcs_db = ARCSDatabaseHandler()

        try:
            while True:
                # 1) Header (1 byte)
                header = conn.recv(1)
                if header != b'\x00':
                    return
                print(f"[USER] Header OK: {header.hex()}")

                # 2) Length (4 bytes)
                length_bytes = conn.recv(4)
                if len(length_bytes) < 4:
                    print("[USER] Incomplete length received")
                    return
                (length,) = struct.unpack('>I', length_bytes)
                print(f"[USER] Payload length: {length}")

                # 3) Command (2 bytes)
                command_raw = conn.recv(2)
                if len(command_raw) < 2:
                    print("[USER] Incomplete command received")
                    return
                command = command_raw.decode('ascii')
                print(f"[USER] Command: {command}")

                # 4) Payload (length bytes)
                if length > 0:
                    data = b''
                    remaining = length
                    while remaining > 0:
                        packet = conn.recv(remaining)
                        if not packet:
                            print(f"[USER] Connection closed early, {remaining} bytes left")
                            return
                        data += packet
                        remaining -= len(packet)
                    payload = json.loads(data.decode('utf-8'))
                    print(f"[USER] Payload: {payload}")
                else:
                    payload = {}

                # ---- 커맨드 처리 ----
                des_coor_list = payload.get("des_coor")
                user_info = payload.get("user_info")
                robot_id = payload.get("robot_id")
                if robot_id is not None:
                    robot_id_queue.put(robot_id)

                # FW 수신 → VD 주기 송신 시작
                if command == "FW":
                    vd_enabled.set()
                    print("[USER][VD] enabled (FW received)")

                # PS/RT/ED 수신 → VD 주기 송신 중지
                if command in ("PS", "RT", "ED"):
                    vd_enabled.clear()
                    print(f"[USER][VD] disabled ({command} received)")

                # VD 이외 커맨드는 즉시 UDP 전송 (vision 바이트는 최신 스냅샷만 읽기)
                if command != "VD":
                    with vision_lock:
                        vision_bytes_snapshot = latest_vision["bytes"]
                    send_to_main_controller_udp(command, des_coor_list, vision_bytes_snapshot)

                # (아래는 기존 로직 유지)
                # CK, LD, FW, RT, PS, FR, ED, KG, AR
                if command in ("LD", "ED"):
                    try:
                        # arcs_db.replace_routes(robot_id, des_coor_list)
                        pass
                    except Exception as e:
                        print(f"[USER][DB ERROR] replace_routes 실패: {e}")
                elif command == "AR":
                    if des_coor_list:
                        pos_x = des_coor_list[0]
                        pos_y = des_coor_list[1]
                        # arcs_db.increment_place_visitor(pos_x, pos_y)

                now = datetime.now()
                if now.hour == 0 and now.minute == 0 and not already_checked.value:
                    # arcs_db.reset_place_visitor_and_total()
                    already_checked.value = True
                if now.hour == 0 and now.minute == 1:
                    already_checked.value = False

        except Exception as e:
            print(f"[USER ERROR] {e}")
    finally:
        vd_enabled.clear()  # 세션 종료 시 VD 중지
        conn.close()
        print(f"[USER] Connection closed from {addr}")


# 커맨드 핸들러 함수 작성
# following, leading, pausing, waiting
    

def send_to_main_controller_udp(command, des_coor_list, vision_data):
    try:
        # command를 ASCII 2바이트로 패킹
        command_bytes = struct.pack('>2s', command.encode('ascii'))

        if command == "VD":
            vision_data_bytes = vision_data
            print(f"[UDP SEND] Sending vision data : {vision_data_bytes}")

            length = len(vision_data_bytes)

            # header + length + payload
            header = b'\x00'
            length_bytes = struct.pack('>I', length)

            full_msg = header + length_bytes + command_bytes + vision_data_bytes

        else:
            # des_coor: [(x, y, angle), ...]
            print(f"[UDP SEND] Sending command: {command}, des_coor_list: {des_coor_list}")
            if des_coor_list is not None:
                des_coor_str = json.dumps(des_coor_list)
                des_coor_bytes = des_coor_str.encode('utf-8')
                length = len(des_coor_bytes)
            else:
                length = 0

            # header + length + payload
            header = b'\x00'
            length_bytes = struct.pack('>I', length)
            if length > 0:
                full_msg = header + length_bytes + command_bytes + des_coor_bytes
            else:
                full_msg = header + length_bytes + command_bytes


        # UDP 전송
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.sendto(full_msg, (main_udp_ip, main_udp_port))
            if des_coor_list is not None:
                print(f"[UDP SEND] {command} sent with {len(des_coor_list)} des_coor points")
            else:
                print(f"[UDP SEND] {command} sent with no destination coordinates")

    except Exception as e:
        print(f"[UDP SEND ERROR] {e}")

def start_main_controller_udp_server(udp_ip, udp_port, running, main_ctrl_data_queue, pos_queue, admin_pos_queue, user_data_queue):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server:
        server.bind((udp_ip, udp_port))
        print(f"[MAIN CONTROLLER] Listening on {udp_ip}:{udp_port}")

        while running.value:
            data, addr = server.recvfrom(65507)  # Buffer size of 4096 bytes
            threading.Thread(target=main_controller_udp_receiver, args=(data, addr, main_ctrl_data_queue, pos_queue, admin_pos_queue, user_data_queue), daemon=True).start()

def main_controller_udp_receiver(data, addr, main_ctrl_data_queue, pos_queue, admin_pos_queue, user_data_queue):
    # print(f"[MAIN CONTROLLER - UDP] Packet received from {addr}")

    try:
        # 1) 첫 1바이트 헤더 검사
        header = data[0:1]
        if header != b'\x00':
            print(f"[MAIN CONTROLLER] Invalid header: {header}")
            return
        # print(f"[CONTROLLER] Header OK: {header.hex()}")

        # 2) 길이
        length = struct.unpack('>I', data[1:5])[0]

        # 3) 명령어 (2바이트)
        command = data[5:7].decode('ascii')
        # print(f"[MAIN CONTROLLER] Command: {command}")

        # 4) payload
        payload = data[7:7+length]

        if command == "MV":
            main_ctrl_data_queue.put(payload)
        elif command == "AR":
            user_data_queue.put(payload)
        else:
            print(f"[MAIN CONTROLLER] Unknown command received: {command}")

        json_data = json.loads(payload.decode('utf-8'))
        print(f"[MAIN CONTROLLER] Json Data: {json_data}")

        current_position = json_data.get("current_position")
        # print(f"[MAIN CONTROLLER] Current Position: {current_position}")
        pos_queue.put(current_position)
        admin_pos_queue.put(current_position)

        loading_data = json_data.get("loadcell")
        if loading_data is None:
            loading = 0
        else:
            loading_data = int(loading_data)
            loading = 1 if loading_data > 0 else 0
        battery = 80
        robot_id = json_data.get("robot_id")

        # arcs_db = ARCSDatabaseHandler()
        # arcs_db.update_arcs_position(robot_id, loading, current_position[0], current_position[1], battery)

    except Exception as e:
        # print(f"[MAIN CONTROLLER - UDP ERROR] {e}")
        pass


# --- add near top ---
class AdminSocket:
    def __init__(self, host, port):
        self.host, self.port = host, port
        self.sock = None
        self.lock = threading.Lock()

    def ensure_connected(self):
        while self.sock is None:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.connect((self.host, self.port))
                self.sock = s
                print("[AdminSock] Connected")
            except Exception as e:
                # print(f"[AdminSock] Reconnect in 1s: {e}")
                time.sleep(1)

    def send_packet(self, command:str, payload_bytes:bytes):
        # 공통 프레이밍: header(1) + length(4) + command(2) + payload
        header = b'\x00'
        length_bytes = struct.pack('>I', len(payload_bytes))
        cmd_bytes = struct.pack('>2s', command.encode('ascii'))
        packet = header + length_bytes + cmd_bytes + payload_bytes
        with self.lock:
            self.ensure_connected()
            try:
                self.sock.sendall(packet)
                print(f"[AdminSock] Packet sent: {packet}")
            except Exception as e:
                print(f"[AdminSock] send failed, retry: {e}")
                try:
                    self.sock.close()
                except:
                    pass
                self.sock = None
                # 1회 재시도
                self.ensure_connected()
                self.sock.sendall(packet)



def start_admin_tcp_server(tcp_ip, admin_tcp_port, running, admin_pos_queue, robot_id_queue):
    admin_sock = AdminSocket(admin_ip, admin_port)
    # ★ CP 송신 스레드: accept 루프 바깥에서 1회만 시작
    threading.Thread(
        target=cp_sender_thread,
        args=(admin_sock, admin_pos_queue, robot_id_queue, running),
        daemon=True
    ).start()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.bind((tcp_ip, admin_tcp_port))
        server.listen()
        print(f"[Admin TCP] Listening on {tcp_ip}:{admin_tcp_port}")

        while running.value:
            conn, addr = server.accept()
            threading.Thread(
                target=admin_tcp_receiver,
                args=(conn, addr, admin_sock, admin_pos_queue, robot_id_queue, running),
                daemon=True
            ).start()


def admin_tcp_receiver(conn, addr, admin_sock, admin_pos_queue, robot_id_queue, running):
    print(f"[Admin] Connected from {addr}")
    try:
        while True:
            header = conn.recv(1)
            if header != b'\x00':
                print(f"[ADMIN] Invalid header: {header}")
                return

            length_bytes = conn.recv(4)
            if len(length_bytes) < 4:
                print("[ADMIN] Incomplete length received")
                return
            (length,) = struct.unpack('>I', length_bytes)

            command_raw = conn.recv(2)
            if len(command_raw) < 2:
                print("[ADMIN] Incomplete command received")
                return
            command = command_raw.decode('ascii')
            print(f"[ADMIN] Command: {command}")

            if length > 0:
                data = b''
                remaining = length
                while remaining > 0:
                    packet = conn.recv(remaining)
                    if not packet:
                        print(f"[ADMIN] Connection closed early, {remaining} bytes left")
                        return
                    data += packet
                    remaining -= len(packet)
                json_data = json.loads(data.decode('utf-8'))
            else:
                json_data = None

            if command:
                # ★ 호출 인자 통일(정의와 동일하게 3개만)
                data_to_admin_pc(admin_sock, command, json_data)
    except Exception as e:
        print(f"[Admin ERROR] {e}")
    finally:
        conn.close()
        print(f"[Admin] Connection closed from {addr}")


def cp_sender_thread(admin_sock, admin_pos_queue, robot_id_queue, running):
    last_robot_id = 1  # 기본값
    while running.value:
        try:
            # ★ robot_id 안전 획득
            try:
                last_robot_id = robot_id_queue.get_nowait()
            except queue.Empty:
                pass

            if not admin_pos_queue.empty():
                current_position = admin_pos_queue.get()
                payload_dict = {"robot_id": last_robot_id, "current_position": current_position}
                payload_bytes = json.dumps(payload_dict).encode('utf-8')
                admin_sock.send_packet("CP", payload_bytes)
                print("[Admin CP Sender] CP sent")
        except Exception as e:
            print(f"[Admin CP Sender ERROR] {e}")
        finally:
            # ★ 무조건 슬립해 CPU 스핀 방지
            time.sleep(2)


def data_to_admin_pc(admin_sock, command, json_data):
    try:
        arcs_db = ARCSDatabaseHandler()

        if command == "LG":
            ok = arcs_db.verify_admin_login(json_data["id"], json_data["password"])
            payload = b'\x01' if ok else b'\x00'
            admin_sock.send_packet("LG", payload)
            print(f"[Admin] Login {'success' if ok else 'fail'} for {json_data['id']}")

        elif command == "RS":
            robot_id = json_data.get("robot_id")
            arcs_list = [] if robot_id in (2,3,4,5,6) else arcs_db.get_arcs(robot_id)
            admin_sock.send_packet("RS", json.dumps(arcs_list).encode('utf-8'))
            print("[Admin] RS sent")

        elif command == "UI":
            robot_id = json_data.get("robot_id")
            user_name = json_data.get("name")
            ticket = json_data.get("ticket")
            created = json_data.get("created")
            user_infos = arcs_db.get_users(robot_id, user_name, ticket, created)
            for u in user_infos:
                if "created" in u and u["created"] is not None:
                    u["created"] = u["created"].strftime("%Y-%m-%d %H:%M:%S")
            admin_sock.send_packet("UI", json.dumps(user_infos).encode('utf-8'))
            print("[Admin] UI sent")
    except Exception as e:
        print(f"[Admin ERROR] {e}")
            

# User PC로 데이터 전송
def data_to_user_pc(user_ip, user_port, running, main_ctrl_data_queue, llm_data_queue, user_data_queue, distance_queue):
    user_conn = None
    while running.value:
        # 1. 연결이 끊겨 있으면 재연결 시도
        if user_conn is None:
            # print("[User] Attempting to connect to User PC...")
            try:
                user_conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                user_conn.connect((user_ip, user_port))
                print("[User] Connected to User PC")
            except (ConnectionRefusedError, OSError) as e:
                # print(f"[User] Waiting to reconnect to User PC... {e}")
                user_conn = None
                time.sleep(2)
                continue  # 재시도
        
        # 2. 연결된 경우 데이터 전송
        try:
            header = b'\x00'
            oo_command = "OO"
            xx_command = "XX"

            # 3. 거리 데이터 처리
            if not distance_queue.empty():
                distance = distance_queue.get() if not distance_queue.empty() else 0
                # print(f"[User] Distance from queue: {distance}")
            if not main_ctrl_data_queue.empty():
                raw_oo_json_bytes = main_ctrl_data_queue.get()
                oo_json_data = json.loads(raw_oo_json_bytes.decode('utf-8'))
                print(f"[User] OO Data: {oo_json_data}")
                dist_state = 0 if distance <= 1.5 else 1
                oo_json_data["distance"] = dist_state
                print(f"[User] distance: {distance}, distance_state: {dist_state}")  # 디버깅용 출력
                oo_json_bytes = json.dumps(oo_json_data).encode('utf-8')

                oo_length_bytes = struct.pack('>I', len(oo_json_bytes))
                oo_command_bytes = struct.pack('>2s', oo_command.encode('ascii'))
                oo_packet = header + oo_length_bytes + oo_command_bytes + oo_json_bytes

                user_conn.sendall(oo_packet)

            if not llm_data_queue.empty():
                xx_json_bytes = llm_data_queue.get()
                xx_length_bytes = struct.pack('>I', len(xx_json_bytes))
                xx_command_bytes = struct.pack('>2s', xx_command.encode('ascii'))
                xx_packet = header + xx_length_bytes + xx_command_bytes + xx_json_bytes
                user_conn.sendall(xx_packet)

            if not user_data_queue.empty():
                command = "AR"
                json_bytes = user_data_queue.get()
                length_bytes = struct.pack('>I', len(json_bytes))
                command_bytes = struct.pack('>2s', command.encode('ascii'))
                packet = header + length_bytes + command_bytes + json_bytes
                user_conn.sendall(packet)

            if (main_ctrl_data_queue.empty() and llm_data_queue.empty() and user_data_queue.empty()):
                user_conn.sendall(b'')   # ← 여기서 끊김 감지
                

        except Exception as e:
            print(f"[User ERROR] {e}")
            try: user_conn.close()
            except: pass
            user_conn = None  # 연결 끊김 표시
            continue  # 재시도


if __name__ == "__main__":
    multiprocessing.set_start_method('spawn', force=True)
    print("현재 start method:", multiprocessing.get_start_method())

    running = Value('b', True)  # 'b' means boolean (byte)
    already_checked = Value('b', False)

    # Queue
    llm_data_queue = Queue()
    vision_data_queue = Queue(maxsize=10)

    main_ctrl_data_queue = Queue()

    user_data_queue = Queue()

    pos_queue = Queue()
    admin_pos_queue = Queue()

    robot_id_queue = Queue()
    distance_queue = Queue(maxsize=10)

    p1 = Process(target=start_vision_tcp_server, args=(tcp_ip, vision_tcp_port, running, vision_data_queue, distance_queue))
    p2 = Process(target=start_llm_tcp_server, args=(tcp_ip, llm_tcp_port, running, llm_data_queue, pos_queue))
    p3 = Process(target=start_user_tcp_server, args=(tcp_ip, user_tcp_port, running, vision_data_queue, robot_id_queue, already_checked))
    p4 = Process(target=start_admin_tcp_server, args=(tcp_ip, admin_tcp_port, running, admin_pos_queue, robot_id_queue))
    p5 = Process(target=start_main_controller_udp_server, args=(udp_ip, udp_port, running, main_ctrl_data_queue, pos_queue, admin_pos_queue, user_data_queue))
    p6 = Process(target=data_to_user_pc, args=(user_ip, user_port, running, main_ctrl_data_queue, llm_data_queue, user_data_queue, distance_queue))

    processes = [p1, p2, p3, p4, p5, p6]

    for p in processes:
        p.start()

    # 메인 루프 유지
    try:
        while running.value:
            time.sleep(1)
    except KeyboardInterrupt:
        running.value = False
        print("Shutting down...")

    for p in processes:
        p.join()