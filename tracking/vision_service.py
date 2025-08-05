import multiprocessing.process
import socket
import queue
import threading

import cv2
import time
import json
import socket
import struct
import numpy as np
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort
from scipy.spatial.distance import cosine

import torch
import torchvision.models as models
import torchvision.transforms as transforms
import torch.nn as nn
from torchvision.models import resnet18, ResNet18_Weights

import multiprocessing
from multiprocessing import Process, Queue, Manager

UDP_SERVER_IP = "0.0.0.0"
UDP_SERVER_PORT = 54321

TCP_IP = "192.168.0.30"
TCP_PORT = 12345

def udp_video_receiver(frame_queue):
    udp_server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_server.bind((UDP_SERVER_IP, UDP_SERVER_PORT))
    print(f"[PC2] UDP Server listening on {UDP_SERVER_IP}:{UDP_SERVER_PORT}")
    
    try:
        while True:
            try:
                data, addr = udp_server.recvfrom(65535)
                np_data = np.frombuffer(data, dtype=np.uint8)
                frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)

                frame_queue.put(frame, block=False)
                
            except KeyboardInterrupt:
                print("[PC2] KeyboardInterrupt detected. Exiting...")
                break
            except Exception as e:
                print(f"[UDP ERROR] {e}")
                continue

    finally:
        print("[PC2] Closing UDP server and destroying windows.")
        udp_server.close()

class Yolo_Deepsort():
    def __init__(self):
        # ------------------------------
        # CONFIGURATION
        # ------------------------------
        self.CAMERA_FRONT = 1
        self.CAMERA_BACK = 0
        self.MODE = "leading"  # or "following"
        self.CAMERA_ID = self.CAMERA_BACK if self.MODE == "leading" else self.CAMERA_FRONT

        self.send_queue = queue.Queue(maxsize=30)

        self.LOST_TIMEOUT = 2.5
        self.COSINE_THRESHOLD = 0.175
        self.MAX_FEATURES = 5

        # ------------------------------
        # INITIALIZE MODELS
        # ------------------------------
        self.model = YOLO("/home/john/dev_ws/final_project/src/yolov8n.pt")
        self.tracker = DeepSort()

        # ------------------------------
        # STATE VARIABLES
        # ------------------------------

        # self.user_feature_list = []  # 등록 시 여러 feature 저장
        self.user_feature = None     # 평균 feature
        self.active_id = None
        self.last_seen = 0
        self.last_user_center = None

        self.SKIP_EVERY_N_FRAMES = 5
        self.frame_count = 0

        # ------------------------------
        # INITIALIZE MODELS
        # ------------------------------
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((128, 64)),  # ReID input size
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ])

        # 1. Device 설정
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"[INFO] Using device: {self.device}")

        # 2. ResNet18 기반 ReID 모델 정의
        self.backbone = models.resnet18(weights=ResNet18_Weights.DEFAULT)
        self.reid_model = nn.Sequential(*list(self.backbone.children())[:-1])  
        # resnet50: [B, 2048, 1, 1] resnet18: [B, 512, 1, 1]
        self.reid_model.add_module("flatten", nn.Flatten())              # [B, 2048], [B, 512]
        self.reid_model = self.reid_model.to(self.device)
        self.reid_model.eval()


    # Dummy function to get Lidar distance (to be replaced by actual Lidar input)
    def get_lidar_distance(self):
        return 1.2  # meters (example value)

    def is_obstacle_ahead(self, threshold=0.6):
        return self.get_lidar_distance() < threshold

    @staticmethod
    def send_loop(send_queue):
        while True:
            try:
                data = send_queue.get()
                if data is None:
                    continue

                # JSON 직렬화
                json_bytes = json.dumps(data).encode('utf-8')
                json_length = len(json_bytes)

                # Header + Length(4바이트) + Payload
                header = b'\x00'
                length_bytes = struct.pack('>I', json_length)  # Big-endian 4바이트 정수

                packet = header + length_bytes + json_bytes

                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.connect((TCP_IP, TCP_PORT))
                    s.sendall(packet)

                # 디버깅용 출력
                # print(f"[TCP SEND] header: 0x00, length: {json_length}, payload: {data}")

            except Exception as e:
                print(f"[TCP ERROR] {e}")

    # Estimate distance using bbox height (simple heuristic)
    def estimate_distance(self, bbox):
        x1, y1, x2, y2 = bbox
        height = y2 - y1
        if height <= 0:
            return float('inf')
        return round(1.7 * (200 / height), 2)
    
    def extract_feature(self, image):
        if image.shape[0] < 30 or image.shape[1] < 30:
            # print("[SKIP] bbox too small for feature extraction")
            return None

        try:
            tensor = self.transform(image).unsqueeze(0).to(self.device)
            with torch.no_grad():
                feat = self.reid_model(tensor)

            # 확실히 1D로 변환
            feat = feat.squeeze().cpu().numpy().reshape(-1)

            if feat.ndim != 1:
                print(f"[ERROR] Feature has shape {feat.shape}, not 1D")
                return None

            # L2 정규화
            feat = feat / np.linalg.norm(feat)
            return feat

        except Exception as e:
            # print("[ERROR] Feature extraction failed:", e)
            return None
        
    def main_processor(self, frame_queue, send_queue, visualization_queue):
        state = "INIT"        # INIT → ACTIVE → WAITING

        # ------------------------------
        # MAIN LOOP
        # ------------------------------
        while True:
            # print(f"[QUEUE] frame_queue size: {frame_queue.qsize()}")
            frame = frame_queue.get()

            self.frame_count += 1
            if self.frame_count % self.SKIP_EVERY_N_FRAMES != 0:
                continue

            if frame is None:
                continue

            results = self.model(frame, verbose=False)[0] # verbose: 기본 로그 출력 여부
            detections = []

            for box in results.boxes:
                if int(box.cls[0]) == 0:  # person class
                    x1, y1, x2, y2 = map(int, box.xyxy[0])

                    width = x2 - x1
                    height = y2 - y1
                    if width < 30 or height < 30:
                        continue  # 너무 작으면 아예 건너뜀

                    conf = float(box.conf[0])
                    detections.append(([x1, y1, x2 - x1, y2 - y1], conf, 'person'))

            tracks = self.tracker.update_tracks(detections, frame=frame)
            current_time = time.time()
            found_user = False

            bboxes_for_viz = []
            for track in tracks:
                if not track.is_confirmed():
                    continue

                tid = track.track_id
                # fvec = track.get_latest_feature()
                x1, y1, x2, y2 = map(int, track.to_ltrb())
                crop = frame[y1:y2, x1:x2]
                fvec = self.extract_feature(crop)  # feature vector 직접 추출
                bbox = track.to_ltrb()
                cx = int((bbox[0] + bbox[2]) / 2)
                cy = int((bbox[1] + bbox[3]) / 2)
                distance = self.estimate_distance(bbox)
                bbox_color = (0, 255, 0) if tid == self.active_id else (100, 100, 100)

                bboxes_for_viz.append({
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                    "tid": tid,
                    "distance": distance,
                    "color":bbox_color
                })

                # # Bounding Box
                # cv2.rectangle(frame, (x1, y1), (x2, y2), bbox_color, 2)

                # # track ID + 거리 텍스트
                # text = f"ID: {tid}  Dist: {distance}m"
                # cv2.putText(frame, text, (x1, y1 - 10),
                #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, bbox_color, 2)


                # INIT 상태일 경우: 가장 가까운 사람 선택
                if state == "INIT":
                    candidates = []
                    for track in tracks:
                        if not track.is_confirmed():
                            continue

                        tid = track.track_id
                        x1, y1, x2, y2 = map(int, track.to_ltrb())
                        crop = frame[y1:y2, x1:x2]
                        fvec = self.extract_feature(crop)
                        if fvec is None:
                            continue

                        bbox = (x1, y1, x2, y2)
                        distance = self.estimate_distance(bbox)
                        cx = int((x1 + x2) / 2)
                        cy = int((y1 + y2) / 2)

                        candidates.append({
                            'tid': tid,
                            'fvec': fvec,
                            'bbox': bbox,
                            'center': (cx, cy),
                            'distance': distance
                        })

                    if candidates:
                        # 가까운 사람 = bbox height 큰 사람 (또는 estimate_distance 낮은 사람)
                        closest = min(candidates, key=lambda x: x['distance'])

                        self.user_feature = closest['fvec']
                        self.active_id = closest['tid']
                        self.last_user_center = closest['center']
                        self.last_seen = current_time
                        state = "ACTIVE"

                        print(f"[INIT] Registered user ID: {self.active_id}")
                        print(f"[INIT] Stored user_feature[:5] = {self.user_feature[:5]}")
                    else:
                        print("[INIT] No valid candidates found for user registration.")


                elif state == "ACTIVE":
                    if tid == self.active_id:
                        self.last_seen = current_time
                        found_user = True
                        self.last_user_center = (cx, cy)
                        send_queue.put({
                            "bbox": bbox,
                            "center": [cx, cy],
                            "track_id": tid,
                            "distance": distance,
                            # "mode": self.MODE,
                            # "state": "ACTIVE"
                        })

                elif state == "WAITING":
                    # 모든 후보 수집
                    candidates = []
                    for track in tracks:
                        if not track.is_confirmed():
                            continue

                        tid = track.track_id
                        x1, y1, x2, y2 = map(int, track.to_ltrb())
                        bbox = (x1, y1, x2, y2)
                        cx = int((x1 + x2) / 2)
                        cy = int((y1 + y2) / 2)
                        distance = self.estimate_distance(bbox)

                        candidates.append({
                            'tid': tid,
                            'bbox': bbox,
                            'center': (cx, cy),
                            'distance': distance
                        })

                    # 1. 가까운 후보들 중에서 N미터 이하만 필터링
                    nearby_candidates = [c for c in candidates if c['distance'] < 3.0]

                    # 2. 유사도 계산해서 가장 유사한 사람 찾기
                    best_match = None
                    min_similarity = 1.0

                    for cand in nearby_candidates:
                        x1, y1, x2, y2 = cand['bbox']
                        crop = frame[y1:y2, x1:x2]
                        fvec = self.extract_feature(crop)

                        if fvec is None or self.user_feature is None:
                            continue

                        similarity = cosine(fvec, self.user_feature)

                        if similarity < min_similarity:
                            min_similarity = similarity
                            best_match = {**cand, 'similarity': similarity}

                    # 3. 최종 비교
                    if best_match and best_match['similarity'] < self.COSINE_THRESHOLD:
                        self.active_id = best_match['tid']
                        self.last_seen = current_time
                        self.last_user_center = best_match['center']
                        state = "ACTIVE"
                        found_user = True
                        print(f"[RECOVER] User re-identified: ID={self.active_id}, sim={best_match['similarity']:.4f}")
                    else:
                        # print("[WAITING] No match found among nearby candidates.")
                        pass


            if state == "ACTIVE" and not found_user:
                if current_time - self.last_seen > self.LOST_TIMEOUT:
                    state = "WAITING"
                    print("[WAITING] User lost → entering WAITING state")

            if state == "WAITING" and self.is_obstacle_ahead():
                print("[LIDAR] Obstacle ahead → possible user re-approaching")


            visualization_queue.put({
                "frame": frame.copy(),
                "bboxes": bboxes_for_viz,
                "state": state  # 현재 상태 문자열
            })

        #     # Visual feedback
        #     cv2.putText(frame, f"State: {state}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        #     cv2.imshow("Tracking", frame)
        #     if cv2.waitKey(0.01) & 0xFF == 27:
        #         break

        # cv2.destroyAllWindows()

    @staticmethod
    def visualization_worker(visualization_queue):
        while True:
            try:
                data = visualization_queue.get(timeout=1)
            except queue.Empty:
                continue

            frame = data["frame"]
            bboxes = data["bboxes"]
            state = data["state"]


            for bbox in bboxes:
                x1, y1, x2, y2 = bbox["x1"], bbox["y1"], bbox["x2"], bbox["y2"]
                tid = bbox["tid"]
                dist = bbox["distance"]
                color = bbox["color"]

                # Draw bbox
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

                # Draw ID and distance
                label = f"ID: {tid}  Dist: {dist:.1f}m"
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # Draw state text
            cv2.putText(frame, f"State: {state}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow("Tracking", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break

        cv2.destroyAllWindows()


def run_tracking_processor(frame_queue, send_queue, visualization_queue):
    tracking_processor = Yolo_Deepsort()  # 여기서 생성
    tracking_processor.main_processor(frame_queue, send_queue, visualization_queue)  # 메서드 호출



if __name__ == "__main__":
    multiprocessing.set_start_method('spawn', force=True)
    print("현재 start method:", multiprocessing.get_start_method())

    # manager = Manager()

    frame_queue = multiprocessing.Queue()
    send_queue = multiprocessing.Queue()
    visualization_queue = multiprocessing.Queue()


    p1 = multiprocessing.Process(target=udp_video_receiver, args=(frame_queue,), daemon=True)
    p2 = multiprocessing.Process(target=run_tracking_processor, args=(frame_queue, send_queue, visualization_queue), daemon=True)
    p3 = multiprocessing.Process(target=Yolo_Deepsort.send_loop, args=(send_queue,), daemon=True)
    p4 = multiprocessing.Process(target=Yolo_Deepsort.visualization_worker, args=(visualization_queue,), daemon=True)
    
    p1.start()
    p2.start()
    p3.start()
    p4.start()

    p1.join()
    p2.join()
    p3.join()
    p4.join()


    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("[VISION] Shutting down...")