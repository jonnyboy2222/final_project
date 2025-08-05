import socket
import threading
from queue import Queue
from transformers import AutoTokenizer, AutoModelForCausalLM
import torch
import struct
import time

response_queue = Queue()

# TinyLlama 모델 로드
print("[모델 로딩 중...]")
model_name = "TinyLlama/TinyLlama-1.1B-Chat-v1.0"
tokenizer = AutoTokenizer.from_pretrained(model_name)
model = AutoModelForCausalLM.from_pretrained(model_name).half().cuda()
model.eval()
print("[모델 로딩 완료]")

# 프롬프트 기반 응답 생성
def run_llm(user_input: str) -> str:
    prompt = f"""당신은 인천공항에서 작동하는 안내 로봇 "아크"입니다.

당신의 주요 임무는 아래와 같습니다:

1. 목적지 안내:
공항 내부 장소(게이트 1~3, 동/서 화장실, 신세계/현대/롯데 면세점, 편의점, 라운지, 안내데스크, 카페, 한식/중식 식당 등 총 15곳)를 방문하고자 하는 사용자를 안내합니다.
목적지까지의 거리(`distance_to_destination`)와 예상 소요 시간(`eta_minutes`)이 제공됩니다.
현재 위치는 `current_location` 변수로 주어집니다.
사용자의 질문 예시: “게이트 2번 가고 싶어”, “화장실 어딨어?”, “롯데 면세점 가려면 얼마나 걸려?”
이러한 질문에 대해, 목적지까지 얼마나 걸리는지, 어디를 향해 가야 하는지 친절하게 설명하세요.

2. 이동 중 상황 응대:
사용자와의 거리(`user_distance`)가 제공됩니다.
일정 거리 이상(`threshold_distance`) 멀어지면 "너무 멀리 계셔서 잠시 멈추겠습니다"와 같은 안내를 하세요.
반대로 너무 가까울 경우엔 “앞에 계시니 천천히 이동할게요” 등도 가능합니다.

3. 짐 운반:
사용자의 뒤를 따라다니며 짐을 운반합니다.
사용자와 멀어지거나 빨리 움직일 경우 "조금 천천히 가주세요!" 같은 반응도 할 수 있습니다.
일상 대화:
사용자가 "너 이름이 뭐야?", "오늘 날씨 어때?", "심심해" 등의 말을 하면, 유머러스하면서 친절하게 응답하세요.
정보가 부족한 경우에도 “죄송하지만 해당 정보는 제공되지 않네요! :땀_흘리는_웃는_얼굴:”처럼 긍정적이고 부드럽게 반응하세요.

4. 추가 지침:
항상 정중하고 밝은 톤을 유지하세요.
사용자에게 먼저 다가가는 듯한 인상을 주는 것이 중요합니다.
숫자 정보는 명확히 전달하되, 너무 딱딱하지 않게 말해주세요.

친절한 말투로 간결하게 대답하세요.

사용자: {user_input}
로봇:
"""
    inputs = tokenizer(prompt, return_tensors="pt").to("cuda")
    with torch.no_grad():
        outputs = model.generate(**inputs, max_new_tokens=128)
    response = tokenizer.decode(outputs[0], skip_special_tokens=True)
    return response.split("로봇:")[-1].strip()

# 길이 프리픽스 메시지 수신 함수
def recv_with_length(sock):
    header = sock.recv(4)
    if len(header) < 4:
        raise ConnectionError("길이 정보를 수신하지 못했습니다.")
    length = struct.unpack(">I", header)[0]  # Big Endian 4-byte
    data = b""
    while len(data) < length:
        packet = sock.recv(length - len(data))
        if not packet:
            break
        data += packet
    return data.decode('utf-8')

# 길이 프리픽스 메시지 전송 함수
def send_with_length(sock, message: str):
    encoded = message.encode('utf-8')
    length_prefix = struct.pack(">I", len(encoded))  # 4-byte Big Endian
    sock.sendall(length_prefix + encoded)

# 서버1: Client1으로부터 수신 → 처리 → 응답 전송 → Queue에 응답 저장
def start_server1(HOST='0.0.0.0', PORT=54322):
    def handle_client(client_socket, address):
        print(f"[서버1 접속됨] {address}")
        try:
            while True:
                try:
                    text = recv_with_length(client_socket)
                except Exception as e:
                    print(f"[서버1 수신 실패] {e}")
                    break
                print(f"[서버1 수신] {text}")
                reply = run_llm(text)
                print(f"[서버1 응답] {reply}")
                send_with_length(client_socket, reply)
                response_queue.put(reply)
        except Exception as e:
            print(f"[서버1 오류] {e}")
        finally:
            client_socket.close()
            print(f"[서버1 연결 종료] {address}")
    def run_server():
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind((HOST, PORT))
        server.listen(5)
        print(f"[서버1 실행 중] {HOST}:{PORT}")
        while True:
            client_socket, address = server.accept()
            threading.Thread(target=handle_client, args=(client_socket, address), daemon=True).start()
    threading.Thread(target=run_server, daemon=True).start()

# 클라이언트2: 응답을 Server2로 전송 (동일한 포맷)
def start_client2(target_host='192.168.0.30', target_port=12346):
    def run_client():
        while True:
            response = response_queue.get()
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.connect((target_host, target_port))
                    send_with_length(s, response)
                    print(f"[클라이언트2 → 서버2 전송됨] {response}")
            except Exception as e:
                print(f"[클라이언트2 오류] {e}")
    threading.Thread(target=run_client, daemon=True).start()

# 메인 실행
if __name__ == "__main__":
    start_server1(HOST='0.0.0.0', PORT=54322)
    start_client2(target_host='192.168.0.30', target_port=12346)

    try:
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("Shut Down")






# import socket
# import threading
# from transformers import AutoTokenizer, AutoModelForCausalLM
# import torch
# # ======================================
# # ChatSession: 모델 + 대화 상태 관리 클래스
# # ======================================
# class ChatSession:
#     def __init__(self):
#         print("[모델 로딩 중...]")
#         self.tokenizer = AutoTokenizer.from_pretrained("outputs", trust_remote_code=True)
#         self.model = AutoModelForCausalLM.from_pretrained("outputs", trust_remote_code=True).half().cuda()
#         self.model.eval()
#         self.messages = [
#             {"role": "system", "content": "당신은 공항 안내 서비스 로봇입니다."}
#         ]
#         print("[모델 로딩 완료]")

#     def chat(self, user_input: str) -> str:
#         self.messages.append({"role": "user", "content": user_input})
#         input_ids = self.tokenizer.apply_chat_template(self.messages, return_tensors="pt").cuda()
#         output = self.model.generate(input_ids, max_new_tokens=128)
#         response = self.tokenizer.decode(output[0], skip_special_tokens=True)
#         response_text = response.split("assistant")[-1].strip()
#         self.messages.append({"role": "assistant", "content": response_text})
#         return response_text
    
# # ======================================
# # TCP 서버 스레드 (LLM 응답 제공)
# # ======================================
# def start_server(HOST='0.0.0.0', PORT=54322):
#     def handle_client(client_socket, address):
#         print(f"[서버 접속됨] {address}")
#         chat_session = ChatSession()
#         try:
#             while True:
#                 data = client_socket.recv(4096).decode('utf-8').strip()
#                 if not data:
#                     break
#                 print(f"[서버 수신 ← {address}] {data}")
#                 reply = chat_session.chat(data)
#                 client_socket.sendall(reply.encode('utf-8'))
#                 print(f"[서버 응답 → {address}] {reply}")
#         except Exception as e:
#             print(f"[서버 오류] {e}")
#         finally:
#             client_socket.close()
#             print(f"[서버 연결 종료] {address}")
#     def run_server():
#         server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         server.bind((HOST, PORT))
#         server.listen(5)
#         print(f"[서버 시작] {HOST}:{PORT}")
#         while True:
#             client_socket, address = server.accept()
#             threading.Thread(target=handle_client, args=(client_socket, address), daemon=True).start()
#     threading.Thread(target=run_server, daemon=True).start()

# # ======================================
# # TCP 클라이언트 스레드 (명령어 전송)
# # ======================================
# def start_client(HOST='192.168.0.30', PORT=12346):
#     def run_client():
#         try:
#             with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
#                 sock.connect((HOST, PORT))
#                 print(f"[클라이언트 연결됨 → {HOST}:{PORT}]")
#                 while True:
#                     user_input = input("사용자 입력 (exit 입력 시 종료): ").strip()
#                     if user_input.lower() == "exit":
#                         break
#                     sock.sendall(user_input.encode('utf-8'))
#                     response = sock.recv(4096).decode('utf-8')
#                     print(f"서버 응답: {response}\n")
#         except Exception as e:
#             print(f"[클라이언트 오류] {e}")
#     threading.Thread(target=run_client, daemon=False).start()

# # ======================================
# # 메인 실행
# # ======================================
# if __name__ == "__main__":
#     start_server(HOST='0.0.0.0', PORT=54322)
#     start_client(HOST='192.168.0.30', PORT=12346)