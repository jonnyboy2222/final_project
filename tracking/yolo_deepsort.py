import cv2
import time
import json
import socket
import numpy as np
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort
from scipy.spatial.distance import cosine

import torch
import torchvision.models as models
import torchvision.transforms as transforms
import torch.nn as nn

# ------------------------------
# CONFIGURATION
# ------------------------------
CAMERA_FRONT = 1
CAMERA_BACK = 0
MODE = "leading"  # or "following"
CAMERA_ID = CAMERA_BACK if MODE == "leading" else CAMERA_FRONT


# Dummy function to get Lidar distance (to be replaced by actual Lidar input)
def get_lidar_distance():
    return 1.2  # meters (example value)

def is_obstacle_ahead(threshold=0.6):
    return get_lidar_distance() < threshold

# Dummy function to send data to TaskHub
def send_to_taskhub(data):
    print("[SEND]", json.dumps(data))
    # Here you can implement socket or HTTP communication

# Estimate distance using bbox height (simple heuristic)
def estimate_distance(bbox):
    x1, y1, x2, y2 = bbox
    height = y2 - y1
    return round(1.7 * (200 / height), 2)

# # 간단한 ReID feature extractor (ResNet18 기반)
# resnet = models.resnet18(pretrained=True)
# resnet.fc = torch.nn.Identity()
# resnet.eval()

transform = transforms.Compose([
    transforms.ToPILImage(),
    transforms.Resize((128, 64)),  # ReID input size
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
])

# 1. Device 설정
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"[INFO] Using device: {device}")

# 2. ResNet50 기반 ReID 모델 정의
backbone = models.resnet50(pretrained=True)
reid_model = nn.Sequential(*list(backbone.children())[:-1])  # [B, 2048, 1, 1]
reid_model.add_module("flatten", nn.Flatten())              # [B, 2048]
reid_model = reid_model.to(device)
reid_model.eval()

# def extract_feature(image):
#     with torch.no_grad():
#         tensor = transform(image).unsqueeze(0)  # [1, 3, H, W]
#         feat = resnet(tensor)
#         feat = feat.squeeze().numpy()  # shape: (512,)
#         feat = feat / np.linalg.norm(feat)  # normalize
#         return feat
    
def extract_feature(image):
    if image.shape[0] < 10 or image.shape[1] < 10:
        print("[SKIP] bbox too small for feature extraction")
        return None

    try:
        tensor = transform(image).unsqueeze(0).to(device)
        with torch.no_grad():
            feat = reid_model(tensor)

        # 확실히 1D로 변환
        feat = feat.squeeze().cpu().numpy().reshape(-1)

        if feat.ndim != 1:
            print(f"[ERROR] Feature has shape {feat.shape}, not 1D")
            return None

        # L2 정규화
        feat = feat / np.linalg.norm(feat)
        return feat

    except Exception as e:
        print("[ERROR] Feature extraction failed:", e)
        return None


# ------------------------------
# INITIALIZE MODELS
# ------------------------------
model = YOLO("yolov8n.pt")
tracker = DeepSort()
cap = cv2.VideoCapture(CAMERA_ID)

# ------------------------------
# STATE VARIABLES
# ------------------------------
state = "INIT"        # INIT → ACTIVE → WAITING
user_feature_list = []  # 등록 시 여러 feature 저장
user_feature = None     # 평균 feature
active_id = None
last_seen = 0
last_user_center = None

COSINE_THRESHOLD = 0.175
LOST_TIMEOUT = 2.5
MAX_FEATURES = 5

# ------------------------------
# MAIN LOOP
# ------------------------------
while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame, verbose=False)[0] # verbose: 기본 로그 출력 여부
    detections = []

    for box in results.boxes:
        if int(box.cls[0]) == 0:  # person class
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            detections.append(([x1, y1, x2 - x1, y2 - y1], conf, 'person'))

    tracks = tracker.update_tracks(detections, frame=frame)
    current_time = time.time()
    found_user = False

    for track in tracks:
        if not track.is_confirmed():
            continue

        tid = track.track_id
        # fvec = track.get_latest_feature()
        x1, y1, x2, y2 = map(int, track.to_ltrb())
        crop = frame[y1:y2, x1:x2]
        fvec = extract_feature(crop)  # feature vector 직접 추출
        bbox = track.to_ltrb()
        cx = int((bbox[0] + bbox[2]) / 2)
        cy = int((bbox[1] + bbox[3]) / 2)
        distance = estimate_distance(bbox)
        bbox_color = (0, 255, 0) if tid == active_id else (100, 100, 100)

        # Bounding Box
        cv2.rectangle(frame, (x1, y1), (x2, y2), bbox_color, 2)

        # track ID + 거리 텍스트
        text = f"ID: {tid}  Dist: {distance}m"
        cv2.putText(frame, text, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, bbox_color, 2)

        # if state == "INIT":
        #     user_feature = fvec
        #     active_id = tid
        #     state = "ACTIVE"
        #     last_seen = current_time
        #     last_user_center = (cx, cy)
        #     print(f"[INIT] Registered user ID: {tid}")
        #     print(f"[INIT] Stored user_feature[:5] = {user_feature[:5]}")

        # INIT 상태일 경우: 가장 가까운 사람 선택
        if state == "INIT":
            candidates = []
            for track in tracks:
                if not track.is_confirmed():
                    continue

                tid = track.track_id
                x1, y1, x2, y2 = map(int, track.to_ltrb())
                crop = frame[y1:y2, x1:x2]
                fvec = extract_feature(crop)
                if fvec is None:
                    continue

                bbox = (x1, y1, x2, y2)
                distance = estimate_distance(bbox)
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

                user_feature = closest['fvec']
                active_id = closest['tid']
                last_user_center = closest['center']
                last_seen = current_time
                state = "ACTIVE"

                print(f"[INIT] Registered user ID: {active_id}")
                print(f"[INIT] Stored user_feature[:5] = {user_feature[:5]}")
            else:
                print("[INIT] No valid candidates found for user registration.")


        elif state == "ACTIVE":
            if tid == active_id:
                last_seen = current_time
                found_user = True
                last_user_center = (cx, cy)
                send_to_taskhub({
                    "track_id": tid,
                    "center": [cx, cy],
                    "distance": distance,
                    "mode": MODE,
                    "state": "ACTIVE"
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
                distance = estimate_distance(bbox)

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
                fvec = extract_feature(crop)

                if fvec is None or user_feature is None:
                    continue

                similarity = cosine(fvec, user_feature)

                if similarity < min_similarity:
                    min_similarity = similarity
                    best_match = {**cand, 'similarity': similarity}

            # 3. 최종 비교
            if best_match and best_match['similarity'] < COSINE_THRESHOLD:
                active_id = best_match['tid']
                last_seen = current_time
                last_user_center = best_match['center']
                state = "ACTIVE"
                found_user = True
                print(f"[RECOVER] User re-identified: ID={active_id}, sim={best_match['similarity']:.4f}")
            else:
                print("[WAITING] No match found among nearby candidates.")



    if state == "ACTIVE" and not found_user:
        if current_time - last_seen > LOST_TIMEOUT:
            state = "WAITING"
            print("[WAITING] User lost → entering WAITING state")

    if state == "WAITING" and is_obstacle_ahead():
        print("[LIDAR] Obstacle ahead → possible user re-approaching")

    # Visual feedback
    cv2.putText(frame, f"State: {state}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("Tracking", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()