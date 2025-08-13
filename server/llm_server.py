# sudo ip route add 192.168.5.0/24 via 192.168.0.23
# ping 192.168.5.9

import socket
import threading
import struct
import json
import math
import time
from queue import Queue
from transformers import AutoTokenizer, AutoModelForCausalLM
from peft import PeftModel
import torch
import re

response_queue = Queue(maxsize=1)
latest_location = None  # 현재 위치 저장
location_lock = threading.Lock()

# 모델 및 토크나이저 로드
base_model_name = "google/gemma-2b"
model_path = "/home/lee/final_project/ARC_chatbot/gemma2b_lora_finetuned_3"
tokenizer = AutoTokenizer.from_pretrained(base_model_name, local_files_only=True)
base_model = AutoModelForCausalLM.from_pretrained(
    base_model_name,
    local_files_only=True,
    torch_dtype=torch.float16
).cuda()  # GPU 수동 할당

model = PeftModel.from_pretrained(base_model, model_path, local_files_only=True).cuda().eval()
print("[모델 로딩 완료]")

# 목적지 좌표 매핑
dest_coords = {
    "게이트 1번": (5.0, 13.1), "게이트 2번": (2.5, 10.0), "게이트 3번": (5.7, 5.7),
    "화장실 서편": (0.0, 0.0), "화장실 동편": (5.7, 8.7),
    "신세계 면세점": (1.0, 0.0), "롯데 면세점": (2.5, 0.0), "현대 면세점": (4.0, 0.0),
    "한식당": (2.5, 7.3), "중식당": (5.5, 0.0),
    "편의점": (3.5, 11.4), "안내데스크": (4.0, 8.7), "카페": (4.0, 5.7)
}

# 유클리드 거리 계산
def calc_distance(p1, p2):
    return round(math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2), 2)

# 정확히 n바이트 수신
def recv_exact(sock, size):
    data = b""
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise ConnectionError("연결 종료됨")
        data += chunk
    return data

# 데이터 수신: header + length + json
def recv_json(sock):
    _ = recv_exact(sock, 1)  # header
    length = struct.unpack(">I", recv_exact(sock, 4))[0]
    data = recv_exact(sock, length)

    return json.loads(data.decode('utf-8'))

# 데이터 전송: header + length + json
def send_json(sock, obj):
    payload = json.dumps(obj).encode('utf-8')
    header = b'\x00'
    length = struct.pack(">I", len(payload))
    sock.sendall(header + length + payload)

# LLM 실행
def run_llm(question: str, current_location: tuple):
    # 목적지 후보 목록
    place_names = list(dest_coords.keys())

    # 질문에 포함된 장소명 찾기 (간단한 substring matching)
    dest_name = None
    for name in place_names:
        if name in question:
            dest_name = name
            break

    # 목적지 좌표 및 거리 계산
    if dest_name:
        dest_position = dest_coords[dest_name]
        distance = round(calc_distance(current_location, dest_position), 2)
    else:
        dest_position = None
        distance = None

    # 질문 앞에 정보 붙이기
    modified_question = (
        f"[현재 위치: {current_location}, "
        f"목적지명: {dest_name if dest_name else '없음'}, "
        f"목적지 위치: {dest_position if dest_position else '없음'}, "
        f"거리: {distance if distance is not None else '계산 불가'}] "
        + question
    )
    # 프롬프트 구성
    # prompt = (
    #     "너는 공항 안내 서비스 로봇 '아크'야. 사용자 질문과 현재 위치를 참고해서 목적지를 추론해줘.\n"
    #     "장소 목록: " + ", ".join(place_names) + "\n"
    #     "응답 형식: {\"intent\": \"guide_user\", \"from\": \"(x, y)\", \"to\": \"장소명 또는 null\", \"confirmation\": \"메시지\"}\n"
    #     f"질문: {modified_question}\n"
    #     "응답:"
    # )

    prompt = (
        # "다음 질문에 대해 JSON으로만 답하시오. "
        # "형식: {\"intent\": \"guide_user\", \"from\": \"(x, y)\", \"to\": \"장소명 또는 null\", \"confirmation\": \"메시지\"}\n\n"
        # f"장소 목록: {', '.join(place_names)}\n"
        # f"현재 위치: {current_location}\n"
        f"질문: {modified_question}\n"
        "응답:"
    )



    # 모델 실행
    inputs = tokenizer(prompt, return_tensors="pt").to("cuda")
    outputs = model.generate(**inputs, max_new_tokens=256, eos_token_id=tokenizer.eos_token_id)
    decoded = tokenizer.decode(outputs[0], skip_special_tokens=True)
    print("[모델 원문 출력]", repr(decoded))


    # try:
    #     match = re.search(r'\{.*\}', decoded, re.DOTALL)
    #     if not match:
    #         raise ValueError("JSON 형식을 찾지 못함. 모델 출력: " + repr(decoded))
    #     json_str = match.group(0)
    #     response = json.loads(json_str)

    #     confirmation = response.get("confirmation", "안내 메시지 없음")
    #     return {
    #         "destination_location": dest_position,
    #         "response": confirmation
    #     }

    # except Exception as e:
    #     print(f"[LLM 파싱 오류] {e}")
    #     return {
    #         "destination_location": dest_position,
    #         "response": "죄송합니다. 안내를 처리할 수 없습니다."
    #     }


    raw_answer = decoded.strip()
    if "화장실" in raw_answer:
        dest_name = "화장실 서편"  # 또는 동편 판단 로직
        dest_position = dest_coords[dest_name]
        return {
            "destination_location": dest_position,
            "response": f"{dest_name}까지 안내하겠습니다."
        }

    
# 위치 수신 스레드
def start_main_service_receiver(port=23459):
    def run():
        # nonlocal latest_location
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(('0.0.0.0', port))
        server.listen(1)

        print(f"[MainService 위치 수신 대기] {port}")

        while True:
            conn, addr = server.accept()
            try:
                print(f"[MainService 연결됨] {addr}")
                current_position = recv_json(conn)
                print(current_position)
                if current_position is not None:
                    x, y = current_position[0], current_position[1]
                else:
                    x, y = 3.0, 13.0 # charging station
                # print("X : ", x, " Y : ", y)

                global latest_location
                with location_lock:
                    latest_location = (x, y)

                print(f"[위치 업데이트] {latest_location}")

            except Exception as e:
                print(f"[MainService 수신 오류] {e}")

    threading.Thread(target=run, daemon=True).start()
    
# 질문 수신 + LLM 처리
def start_data_sender_receiver(port=54323):
    def handle_client(conn, addr):
        print(f"[DataSender 연결됨] {addr}")
        try:
            data = recv_json(conn)
            question = data.get("question", "")
            with location_lock:
                if latest_location is None:
                    print("[위치 없음 → 질문 무시]")
                    return
                loc = latest_location
            print(f"[질문 수신] {question}")
            response = run_llm(question, loc)
            print(f"[LLM 응답] {response}")
            response_queue.put(response)
        except Exception as e:
            import traceback
            traceback.print_exc()
            print(f"[DataSender 오류] {e}")
            return
        finally:
            conn.close()

    def run():
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(('0.0.0.0', port))
        server.listen(5)
        print(f"[DataSender 질문 수신 대기] {port}")
        while True:
            conn, addr = server.accept()
            threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()
    threading.Thread(target=run, daemon=True).start()

# 응답 전송 클라이언트
class MainServiceClientThread(threading.Thread):
    def __init__(self, host, port, queue: Queue):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.queue = queue
        
    def run(self):
        while True:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                    sock.connect((self.host, self.port))
                    print(f"[MainService 전송 클라이언트 연결됨 → {self.host}:{self.port}]")
                    while True:
                        response = self.queue.get()
                        if response is None:
                            print("[전송 스레드 종료]")
                            break
                        send_json(sock, response)
                        print(f"[응답 전송 완료] {response}")

            except Exception as e:
                print(f"[MainService 전송 오류] {e}")
                time.sleep(2)

# 실행
if __name__ == "__main__":
    start_main_service_receiver(port=23459)
    start_data_sender_receiver(port=54323)
    MainServiceClientThread(host='192.168.0.145', port=12346, queue=response_queue).start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shut Down")
