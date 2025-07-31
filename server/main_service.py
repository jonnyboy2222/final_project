import socket
import struct
import threading
import json
import time
from datetime import datetime

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



admin_ip = '192.168.0.18'      # Admin PC IP
admin_port = 23456             # Admin TCP 포트

user_ip = '192.168.0.30'       # User PC IP
user_port = 23457              # User TCP 포트

main_udp_ip = '192.168.0.100'  # 메인 컨트롤러 IP
main_udp_port = 23458          # 메인 컨트롤러 포트

llm_tcp_ip = "192.168.0.33"    # Chat Service IP
llm_port = 23459               # Chat Service TCP 포트


    
# TCP 데이터 수신
def start_vision_tcp_server(tcp_ip, vision_tcp_port, running, vision_data_queue):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.bind((tcp_ip, vision_tcp_port))
        server.listen()
        print(f"[Vision TCP] Listening on {tcp_ip}:{vision_tcp_port}")

        while running.value:
            conn, addr = server.accept()
            threading.Thread(target=vision_tcp_receiver, args=(conn, addr, vision_data_queue), daemon=True).start()

def vision_tcp_receiver(conn, addr, vision_data_queue):
    print(f"[Vision] Connected from {addr}")
    try:
        # 1) Read exactly 1 byte for the header
        header = conn.recv(1)
        
        if header == b'\x00':
            print(f"header received: {header}")
        else:
            print(f"{header} received: pass")

        
        # 2) Read exactly 4 bytes for the length
        length_bytes = conn.recv(4)
        if len(length_bytes) < 4:
            print("[VISION] Connection closed before length received")
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
                print(f"[VISION] Connection closed with {remaining} bytes left to read")
                return
            data += packet
            remaining -= len(packet)

        # 4) Decode and enqueue
        json_data = json.loads(data.decode('utf-8'))
        print(f"[VISION] Data: {json_data}")

        vision_data_queue.put(data)

    except Exception as e:
        print(f"[VISION ERROR] {e}")

    finally:
        conn.close()

def start_llm_tcp_server(tcp_ip, llm_tcp_port, running, llm_data_queue, pos_queue):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.bind((tcp_ip, llm_tcp_port))
        server.listen()
        print(f"[LLM TCP] Listening on {tcp_ip}:{llm_tcp_port}")

        while running.value:
            conn, addr = server.accept()
            threading.Thread(target=llm_tcp_receiver, args=(conn, addr, llm_data_queue, pos_queue), daemon=True).start()


def llm_tcp_receiver(conn, addr, llm_data_queue, pos_queue):
    print(f"[LLM] Connected from {addr}")
    try:
        # 1) Read exactly 1 byte for the header
        header = conn.recv(1)
        
        if header == b'\x00':
            print(f"header received: {header}")
        else:
            print(f"{header} received: pass")

        
        # 2) Read exactly 4 bytes for the length
        length_bytes = conn.recv(4)
        if len(length_bytes) < 4:
            print("[LLM] Connection closed before length received")
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
                print(f"[LLM] Connection closed with {remaining} bytes left to read")
                return
            data += packet
            remaining -= len(packet)

        # 4) Decode and enqueue
        json_data = data.decode('utf-8')
        print(f"[LLM] Data: {json_data}")
        llm_data_queue.put(data)


        current_pos = pos_queue.get()
        print(f"[LLM] Current Position: {current_pos}")

        current_pos_bytes = json.dumps(current_pos).encode('utf-8')
        send_pos_to_llm(current_pos_bytes)

    except Exception as e:
        print(f"[LLM ERROR] {e}")
    finally:
        conn.close()

def send_pos_to_llm(current_pos_bytes):
    try:
        header = b'\x00'
        length_bytes = struct.pack('>I', len(current_pos_bytes))

        packet = header + length_bytes + current_pos_bytes

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.connect((llm_tcp_ip, llm_port))
            sock.sendall(packet)

    except Exception as e:
        print(f"[LLM TCP SEND ERROR] {e}")


def start_user_tcp_server(tcp_ip, user_tcp_port, running, vision_data_queue, user_name_queue, robot_id_queue, already_checked):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.bind((tcp_ip, user_tcp_port))
        server.listen()
        print(f"[User TCP] Listening on {tcp_ip}:{user_tcp_port}")

        while running.value:
            conn, addr = server.accept()
            threading.Thread(target=user_tcp_receiver, args=(conn, addr, vision_data_queue, user_name_queue, robot_id_queue, already_checked), daemon=True).start()

def user_tcp_receiver(conn, addr, vision_data_queue, user_name_queue, robot_id_queue, already_checked):
    print(f"[USER] Connected from {addr}")
    try:
        arcs_db = ARCSDatabaseHandler()
        # 1. Header (1 byte)
        header = conn.recv(1)
        if header != b'\x00':
            print(f"[USER] Invalid header: {header}")
            return
        print(f"[USER] Header OK: {header.hex()}")

        # 2. Length (4 bytes)
        length_bytes = conn.recv(4)
        if len(length_bytes) < 4:
            print("[USER] Incomplete length received")
            return
        (length,) = struct.unpack('>I', length_bytes)
        print(f"[USER] Payload length: {length}")

        # 3. Command (2 bytes)
        command_raw = conn.recv(2)
        if len(command_raw) < 2:
            print("[USER] Incomplete command received")
            return
        command = command_raw.decode('ascii')
        print(f"[USER] Command: {command}")

        # 4. Payload (length bytes)
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

            # 5. Decode JSON
            payload = json.loads(data.decode('utf-8'))
            print(f"[USER] Payload: {payload}")
            
        else:
            payload = {}

        # 즉시 메인 컨트롤러에 커맨드 전달
        des_coor_list = payload.get("des_coor")
        vision_data = vision_data_queue.get() if not vision_data_queue.empty() else None

        send_to_main_controller_udp(command, des_coor_list, vision_data)

        user_info = payload.get("user_info")
        robot_id = user_info["robot_id"]

        robot_id_queue.put(robot_id)

        if user_info:
            arcs_db.save_user(user_info)

            user_name_queue.put(user_info["name"])

        # CK, LD, FW, RT, PS, FR, ED, KG, AR
        if command in ("LD", "ED"):
            arcs_db.replace_routes(robot_id, des_coor_list)
        elif command == "AR":
            pos_x = des_coor_list[0]
            pos_y = des_coor_list[1]
            arcs_db.increment_place_visitor(pos_x, pos_y)


        now = datetime.now()

        if now.hour == 0 and now.minute == 0 and not already_checked.value:
            arcs_db.reset_place_visitor_and_total()
            already_checked.value = True

        if now.hour == 0 and now.minute == 1:
            already_checked.value = False


    except Exception as e:
        print(f"[USER ERROR] {e}")
    finally:
        conn.close()

# 커맨드 핸들러 함수 작성
# following, leading, pausing, waiting
    

def send_to_main_controller_udp(command, des_coor_list, vision_data):
    try:
        # command를 ASCII 2바이트로 패킹
        command_bytes = struct.pack('>2s', command.encode('ascii'))

        if command == "VD":
            vision_data_bytes = vision_data

            length = len(vision_data_bytes)

            # header + length + payload
            header = b'\x00'
            length_bytes = struct.pack('>I', length)

            full_msg = header + length_bytes + command_bytes + vision_data_bytes

        else:
            # des_coor: [(x, y, angle), ...]
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
            print(f"[UDP SEND] {command} sent with {len(des_coor_list)} des_coor points")

    except Exception as e:
        print(f"[UDP SEND ERROR] {e}")

def start_main_controller_udp_server(udp_ip, udp_port, running, main_ctrl_data_queue, pos_queue, admin_pos_queue, user_data_queue):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.bind((udp_ip, udp_port))
        server.listen()
        print(f"[LLM TCP] Listening on {udp_ip}:{udp_port}")

        while running.value:
            conn, addr = server.accept()
            threading.Thread(target=main_controller_udp_receiver, args=(conn, addr, main_ctrl_data_queue, pos_queue, admin_pos_queue, user_data_queue), daemon=True).start()

def main_controller_udp_receiver(conn, addr, main_ctrl_data_queue, pos_queue, admin_pos_queue, user_data_queue):
    print(f"[MAIN CONTROLLER] Connected from {addr}")
    try:
        arcs_db = ARCSDatabaseHandler()
        # 1) Read exactly 1 byte for the header
        header = conn.recv(1)
        
        if header == b'\x00':
            print(f"header received: {header}")
        else:
            print(f"{header} received: pass")

        
        # 2) Read exactly 4 bytes for the length
        length_bytes = conn.recv(4)
        if len(length_bytes) < 4:
            print("[MAIN CONTROLLER] Connection closed before length received")
            return

        # Unpack as unsigned int, big-endian
        (length,) = struct.unpack('>I', length_bytes)
        # e.g. length = 1234

        # 3) Read exactly 2 bytes for the command
        command_bytes = conn.recv(2)
        command = command_bytes.decode('ascii')
        print(f"[MAIN CONTROLLER] Command: {command}")


        # 4) Read exactly `length` bytes
        if length > 0:
            data = b''
            remaining = length
            while remaining > 0:
                packet = conn.recv(remaining)
                if not packet:
                    print(f"[MAIN CONTROLLER] Connection closed with {remaining} bytes left to read")
                    return
                data += packet
                remaining -= len(packet)


        if command == "MV":
            main_ctrl_data_queue.put(data)
        
        elif command == "AR":
            user_data_queue.put(data)

        else:
            print(f"[USER] Unknown command received: {command}")

        # 4) Decode and enqueue
        json_data = json.loads(data.decode('utf-8'))
        print(f"[MAIN CONTROLLER] Json Data: {json_data}")

        current_position = json_data.get("current_position")
        
        pos_queue.put(current_position)
        admin_pos_queue.put(current_position)

        loading_data = json_data.get("loadcell")
        loading = 1 if loading_data > 0 else 0
        battery = 80
        robot_id = json_data.get("robot_id")

        arcs_db.update_arcs_position(robot_id, loading, current_position[0], current_position[1], battery)

    except Exception as e:
        print(f"[MAIN CONTROLLER ERROR] {e}")
    finally:
        conn.close()

def start_admin_tcp_server(tcp_ip, admin_tcp_port, running, admin_pos_queue, user_name_queue, robot_id_queue):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.bind((tcp_ip, admin_tcp_port))
        server.listen()
        print(f"[Admin TCP] Listening on {tcp_ip}:{admin_tcp_port}")

        while running.value:
            conn, addr = server.accept()
            threading.Thread(target=admin_tcp_receiver, args=(conn, addr, admin_ip, admin_port, admin_pos_queue, user_name_queue, robot_id_queue), daemon=True).start()

def admin_tcp_receiver(conn, addr, admin_ip, admin_port, admin_pos_queue, user_name_queue, robot_id_queue):
    print(f"[Admin] Connected from {addr}")
    try:
        # 1. Header (1 byte)
        header = conn.recv(1)
        if header != b'\x00':
            print(f"[USER] Invalid header: {header}")
            return
        print(f"[USER] Header OK: {header.hex()}")

        # 2. Length (4 bytes)
        length_bytes = conn.recv(4)
        if len(length_bytes) < 4:
            print("[USER] Incomplete length received")
            return
        (length,) = struct.unpack('>I', length_bytes)
        print(f"[USER] Payload length: {length}")

        # 3. Command (2 bytes)
        command_raw = conn.recv(2)
        if len(command_raw) < 2:
            print("[USER] Incomplete command received")
            return
        command = command_raw.decode('ascii')
        print(f"[USER] Command: {command}")

        # 4. Payload (length bytes)
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

            json_data = json.loads(data.decode('utf-8'))
            print(f"[Admin] Login info: {json_data}")
            
        data_to_admin_pc(admin_ip, admin_port, command, json_data, admin_pos_queue, user_name_queue, robot_id_queue)

        

    except Exception as e:
        print(f"[Admin ERROR] {e}")
    finally:
        conn.close()


# Admin PC로 데이터 전송
def data_to_admin_pc(admin_ip, admin_port, command, json_data, admin_pos_queue, user_name_queue, robot_id_queue):
    try:
        arcs_db = ARCSDatabaseHandler()
        admin_conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        admin_conn.connect((admin_ip, admin_port))
        print("[Admin] Connected to Admin PC")

        header = b'\x00'
        login_success = b'\x01'
        login_fail = b'\x00'

        cp_command = "CP"

        if not robot_id_queue.empty():
            robot_id = robot_id_queue.get()
        else:
            robot_id = None


        
        if not admin_pos_queue.empty():
            current_position = admin_pos_queue.get()

            payload_dict = {
                "robot_id": robot_id,
                "current_position": current_position
            }

            cp_json_bytes = json.dumps(payload_dict).encode('utf-8')
            cp_length_bytes = struct.pack('>I', len(cp_json_bytes))
            cp_command_bytes = struct.pack('>2s', cp_command.encode('ascii'))

            cp_packet = header + cp_length_bytes + cp_command_bytes + cp_json_bytes
            admin_conn.sendall(cp_packet)


        if command == "LG":
            # db 검증 수행 후 success or fail
            login = arcs_db.verify_admin_login(json_data["id"], json_data["password"])

            lg_length_bytes = struct.pack('>I', 1)
            lg_command_bytes = struct.pack('>2s', command.encode('ascii'))

            if login == "Login success":
                lg_packet = header + lg_length_bytes + lg_command_bytes + login_success
                admin_conn.sendall(lg_packet)
            else:
                lg_packet = header + lg_length_bytes + lg_command_bytes + login_fail
                admin_conn.sendall(lg_packet)

        elif command == "RS":
            arcs_list = arcs_db.get_arcs()

            # user, task, loading, using, battery, pos_x, pos_y, status_time
            rs_dict = {
                "user": arcs_list[1],
                "task": arcs_list[2],
                "loading": arcs_list[3],
                "using": arcs_list[4],
                "battery": arcs_list[5],
                "current_position": (arcs_list[6], arcs_list[7]),
                "status_time": arcs_list[8]
            }
            
            rs_json_bytes = json.dumps(rs_dict).encode('utf-8')
            rs_length_bytes = struct.pack('>I', len(rs_json_bytes))
            rs_command_bytes = struct.pack('>2s', command.encode('ascii'))

            rs_packet = header + rs_length_bytes + rs_command_bytes + rs_json_bytes
            admin_conn.sendall(rs_packet)

        elif command == "UI":
            # name, ticket, robot id
            user_name = user_name_queue.get()
            user_info = arcs_db.get_users(user_name)

            # name, boarding, departure, gate, sex, age, seat, from, to
            user_info_dict = {
                "name": user_info[0],
                "boarding": user_info[1],
                "departure": user_info[2],
                "gate": user_info[3],
                "sex": user_info[4],
                "age": user_info[5],
                "seat": user_info[6],
                "from": user_info[7],
                "to": user_info[8]
            }

            ui_json_bytes = json.dumps(user_info_dict).encode('utf-8')
            ui_length_bytes = struct.pack('>I', len(ui_json_bytes))
            ui_command_bytes = struct.pack('>2s', command.encode('ascii'))

            ui_packet = header + ui_length_bytes + ui_command_bytes + ui_json_bytes
            admin_conn.sendall(ui_packet)
        

    except Exception as e:
        print(f"[Admin ERROR] {e}")
        if admin_conn:
            admin_conn.close()
            admin_conn = None
        time.sleep(1)
            

# User PC로 데이터 전송
def data_to_user_pc(user_ip, user_port, running, main_ctrl_data_queue, llm_data_queue, user_data_queue):

    user_conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    user_conn.connect((user_ip, user_port))
    print("[User] Connected to User PC")
    
    while running.value:
        try:
            header = b'\x00'
            oo_command = "OO"
            xx_command = "XX"

            if main_ctrl_data_queue.empty():
                continue

            else: # not empty
                oo_json_bytes = main_ctrl_data_queue.get()

                # Header + Length(4바이트) + Payload
                oo_length_bytes = struct.pack('>I', len(oo_json_bytes))  # Big-endian 4바이트 정수
                oo_command_bytes = struct.pack('>2s', oo_command.encode('ascii')) # command를 ASCII 2바이트로 패킹
            
                oo_packet = header + oo_length_bytes + oo_command_bytes + oo_json_bytes

                user_conn.sendall(oo_packet)

            if llm_data_queue.empty():
                continue

            else: # not empty
                xx_json_bytes = llm_data_queue.get()

                # Header + Length(4바이트) + Payload
                xx_length_bytes = struct.pack('>I', len(xx_json_bytes))  # Big-endian 4바이트 정수
                xx_command_bytes = struct.pack('>2s', xx_command.encode('ascii')) # command를 ASCII 2바이트로 패킹
            
                xx_packet = header + xx_length_bytes + xx_command_bytes + xx_json_bytes

                user_conn.sendall(xx_packet)

            if user_data_queue.empty():
                continue

            else: # not empty
                command = "AR"
                json_bytes = user_data_queue.get()

                # Header + Length(4바이트) + Payload
                length_bytes = struct.pack('>I', len(json_bytes))  # Big-endian 4바이트 정수
                command_bytes = struct.pack('>2s', command.encode('ascii')) # command를 ASCII 2바이트로 패킹
            
                packet = header + length_bytes + command_bytes + json_bytes

                user_conn.sendall(packet)

            
        except Exception as e:
            print(f"[User ERROR] {e}")
            if user_conn:
                user_conn.close()
                user_conn = None
            time.sleep(1)


if __name__ == "__main__":
    multiprocessing.set_start_method('spawn', force=True)
    print("현재 start method:", multiprocessing.get_start_method())

    running = Value('b', True)  # 'b' means boolean (byte)
    already_checked = Value('b', False)

    # Queue
    llm_data_queue = Queue()
    vision_data_queue = Queue()

    main_ctrl_data_queue = Queue()

    user_data_queue = Queue()
    user_name_queue = Queue()

    pos_queue = Queue()
    admin_pos_queue = Queue()

    robot_id_queue = Queue()

    p1 = Process(target=start_vision_tcp_server, args=(tcp_ip, vision_tcp_port, running, vision_data_queue))
    p2 = Process(target=start_llm_tcp_server, args=(tcp_ip, llm_tcp_port, running, llm_data_queue, pos_queue))
    p3 = Process(target=start_user_tcp_server, args=(tcp_ip, user_tcp_port, running, vision_data_queue, user_name_queue, robot_id_queue, already_checked))
    p4 = Process(target=start_admin_tcp_server, args=(tcp_ip, admin_tcp_port, running, admin_pos_queue, user_name_queue, robot_id_queue))
    p5 = Process(target=start_main_controller_udp_server, args=(udp_ip, udp_port, running, main_ctrl_data_queue, pos_queue, admin_pos_queue, user_data_queue))
    p6 = Process(target=data_to_user_pc, args=(user_ip, user_port, running, main_ctrl_data_queue, llm_data_queue, user_data_queue))

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