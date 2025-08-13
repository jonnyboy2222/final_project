
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
import torch.nn as nn
import multiprocessing
from multiprocessing import Process, Queue, Manager

# --------------------
# Configurable params
# --------------------
UDP_SERVER_IP = "0.0.0.0"
UDP_SERVER_PORT = 54322

TCP_IP = "192.168.0.145"
TCP_PORT = 12345

# Resize incoming frames to this (width, height).
# Smaller => faster inference but lower accuracy. Adjust to taste.
FRAME_RESIZE = (320, 320)

# Multiprocessing queue max size: keeps memory bounded.
FRAME_QUEUE_MAXSIZE = 6

# YOLO inference image size (shorter side). If you already resize frames, this can match that width.
YOLO_IMG_SIZE = 320

# --------------------
# Utility
# --------------------
def get_latest_frame_from_queue(mp_queue, timeout=0.5):
    """
    Get the most recent frame from a multiprocessing.Queue.
    This drains older frames so the processor always works on the latest frame.
    """
    try:
        frame = mp_queue.get(timeout=timeout)
    except Exception:
        return None

    # Drain older frames (non-blocking) to keep the latest one
    while True:
        try:
            frame = mp_queue.get_nowait()
        except Exception:
            break
    return frame

# --------------------
# UDP receiver with backpressure (drop-oldest-on-full)
# --------------------
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

                if frame is None:
                    continue

                # Resize immediately to reduce memory & inference cost.
                if FRAME_RESIZE is not None:
                    try:
                        frame = cv2.resize(frame, FRAME_RESIZE, interpolation=cv2.INTER_LINEAR)
                    except Exception:
                        pass

                # Try to push without blocking. If full, drop oldest then push.
                try:
                    frame_queue.put_nowait(frame)
                except Exception:
                    # Queue is full or other error -> drop one oldest and try again.
                    try:
                        frame_queue.get_nowait()
                    except Exception:
                        pass
                    try:
                        frame_queue.put_nowait(frame)
                    except Exception as e:
                        # If still failing, skip this frame
                        # (Avoid spamming prints; keep a minimal debug)
                        # print("[UDP] Frame put failed even after drop:", e)
                        pass

            except KeyboardInterrupt:
                print("[PC2] KeyboardInterrupt detected. Exiting...")
                break
            except Exception as e:
                # Non-fatal errors shouldn't stop the receiver
                print(f"[UDP ERROR] {e}")
                continue

    finally:
        print("[PC2] Closing UDP server.")
        udp_server.close()

# --------------------
# Main tracking class
# --------------------
class Yolo_Deepsort():
    def __init__(self):
        # ------------------------------
        # CONFIGURATION
        # ------------------------------
        self.CAMERA_FRONT = 1
        self.CAMERA_BACK = 0
        self.MODE = "leading"  # or "following"
        self.CAMERA_ID = self.CAMERA_BACK if self.MODE == "leading" else self.CAMERA_FRONT

        # Small send queue (keeps latest message only)
        self.send_queue = queue.Queue(maxsize=30)

        self.LOST_TIMEOUT = 2.5
        self.COSINE_THRESHOLD = 0.25
        self.MAX_FEATURES = 5

        # ------------------------------
        # INITIALIZE MODELS
        # ------------------------------
        # Path to your weights (keep same as before)
        self.model = YOLO("/home/john/dev_ws/final_project/src/yolov8n.pt")

        # Try to optimize model for GPU if available
        self.use_gpu = torch.cuda.is_available()
        if self.use_gpu:
            try:
                # Attempt to set internal model to half and move to CUDA.
                # ultralytics exposes underlying torch model as model.model
                try:
                    self.model.model.half()
                except Exception:
                    pass
                try:
                    self.model.to('cuda:0')
                except Exception:
                    pass
                print("[INFO] YOLO: configured for CUDA (half precision attempted).")
            except Exception as e:
                print("[INFO] YOLO GPU configuration failed:", e)
        else:
            print("[INFO] YOLO: using CPU.")

        # DeepSort tracker
        self.tracker = DeepSort(max_age=30)  # adjust max_age if needed

        # ------------------------------
        # STATE VARIABLES
        # ------------------------------
        self.user_feature = None     # 평균 feature
        self.active_id = None
        self.last_seen = 0
        self.last_user_center = None

        # Frame skip: process only 1 every N frames (after draining to latest)
        self.SKIP_EVERY_N_FRAMES = 5
        self.frame_count = 0

        # ------------------------------
        # ReID model (ResNet18 backbone)
        # ------------------------------
        # Keep reid model in float32 for robustness
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"[INFO] Using device for ReID: {self.device}")

        backbone = models.resnet18(weights=models.ResNet18_Weights.DEFAULT)
        reid_head = nn.Sequential(*list(backbone.children())[:-1])  # remove classifier
        reid_head.add_module("flatten", nn.Flatten())
        self.reid_model = reid_head.to(self.device)
        self.reid_model.eval()

        # Pre-compute normalization (numpy-friendly)
        self.reid_mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        self.reid_std = np.array([0.229, 0.224, 0.225], dtype=np.float32)

    # Dummy function (same)
    def get_lidar_distance(self):
        return 1.2

    def is_obstacle_ahead(self, threshold=0.6):
        return self.get_lidar_distance() < threshold

    @staticmethod
    def send_loop(send_queue):
        sender = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sender.settimeout(10)
        try:
            sender.connect((TCP_IP, TCP_PORT))
            print(f"[TCP] Connected to {TCP_IP}:{TCP_PORT}")
        except Exception as e:
            print(f"[TCP] Connect failed: {e}")
            return

        try:
            while True:
                try:
                    data = None
                    try:
                        data = send_queue.get(timeout=2)
                    except Exception:
                        pass

                    if data is None:
                        continue

                    json_bytes = json.dumps(data).encode('utf-8')
                    json_length = len(json_bytes)
                    header = b'\x00'
                    length_bytes = struct.pack('>I', json_length)
                    packet = header + length_bytes + json_bytes

                    sender.sendall(packet)
                    print(f"[TCP SEND] header: 0x00, length: {json_length}, payload: {data}")
                    # throttle send slightly (avoid extremely frequent sends)
                    time.sleep(0.01)

                except Exception as e:
                    # Keep loop alive; re-try connect on errors
                    # print(f"[TCP ERROR] {e}")
                    time.sleep(1)
                    continue
        except KeyboardInterrupt:
            sender.close()
            print("[TCP] KeyboardInterrupt detected. Exiting...")

    def estimate_distance(self, bbox):
        x1, y1, x2, y2 = bbox
        height = y2 - y1
        if height <= 0:
            return float('inf')
        
        return round(1.7 * (200 / height), 2)

    def extract_feature(self, image):
        """
        Faster feature extraction:
        - avoid PIL/torchvision transforms;
        - use cv2 resize + numpy -> tensor pipeline.
        """
        try:
            h, w = image.shape[:2]
            if h < 30 or w < 30:
                return None

            # Resize to ReID input (width=64, height=128)
            reid_w, reid_h = 64, 128
            try:
                img = cv2.resize(image, (reid_w, reid_h), interpolation=cv2.INTER_LINEAR)
            except Exception:
                return None

            # BGR -> RGB, float32, scale
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0

            # normalize
            img = (img - self.reid_mean) / self.reid_std

            # HWC -> CHW
            img = img.transpose(2, 0, 1)
            tensor = torch.from_numpy(img).unsqueeze(0).to(self.device)  # [1,3,128,64]
            tensor = tensor.float()

            with torch.no_grad():
                feat = self.reid_model(tensor)
            feat = feat.squeeze().cpu().numpy().reshape(-1)

            # protect against zero-vector
            norm = np.linalg.norm(feat)
            if norm == 0 or np.isnan(norm):
                return None
            feat = feat / norm
            return feat
        except Exception as e:
            # print("[ERROR] Feature extraction failed:", e)
            return None

    def main_processor(self, frame_queue, send_queue, visualization_queue):
        state = "INIT"        # INIT → ACTIVE → WAITING
        last_print = time.time()

        while True:
            # Always pick the latest frame (drop older ones)
            frame = get_latest_frame_from_queue(frame_queue, timeout=0.5)
            if frame is None:
                continue

            # count for frame skipping
            self.frame_count += 1
            if self.frame_count % self.SKIP_EVERY_N_FRAMES != 0:
                continue

            # Inference: prefer to pass small frames (we already resized at receive)
            try:
                # Try to call ultralytics with faster options; fallback to basic call if args not supported.
                try:
                    results = self.model(frame, imgsz=YOLO_IMG_SIZE,
                                         device=0 if self.use_gpu else 'cpu',
                                         half=self.use_gpu,
                                         verbose=False)[0]
                except Exception:
                    results = self.model(frame, verbose=False)[0]
            except Exception as e:
                # Skip frame on model error
                # print("[YOLO ERROR]", e)
                continue

            detections = []
            for box in results.boxes:
                try:
                    if int(box.cls[0]) == 0:  # person
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        width = x2 - x1
                        height = y2 - y1
                        if width < 30 or height < 30:
                            continue
                        conf = float(box.conf[0])
                        detections.append(([x1, y1, x2 - x1, y2 - y1], conf, 'person'))
                except Exception:
                    continue

            tracks = self.tracker.update_tracks(detections, frame=frame)
            current_time = time.time()
            found_user = False

            bboxes_for_viz = []
            # Collect confirmed tracks and do reid on demand
            for track in tracks:
                if not track.is_confirmed():
                    continue
                try:
                    tid = track.track_id
                    x1, y1, x2, y2 = map(int, track.to_ltrb())
                    # safe bbox clipping
                    x1 = max(0, x1); y1 = max(0, y1)
                    x2 = min(frame.shape[1]-1, x2); y2 = min(frame.shape[0]-1, y2)
                    if x2 <= x1 or y2 <= y1:
                        continue

                    crop = frame[y1:y2, x1:x2]
                    fvec = self.extract_feature(crop)
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

                    # INIT state: find closest and register user once
                    if state == "INIT":
                        # use existing tracks to select candidate with largest bbox height (closest)
                        # we perform this outside of loop for efficiency; keep simple here
                        pass

                    elif state == "ACTIVE":
                        if tid == self.active_id:
                            self.last_seen = current_time
                            found_user = True
                            self.last_user_center = (cx, cy)
                            send_queue.put({
                                "center_point": [float(cx), float(cy)],
                                "distance": float(distance),
                            })
                    elif state == "WAITING":
                        # WAITING logic same as before; do candidate matching below
                        send_queue.put({
                                "center_point": [],
                                "distance": float(1),
                            })
                        # pass

                except Exception:
                    continue

            # Additional INIT and WAITING logic (kept similar to original but performed after track collection)
            # Build simple candidate list when in INIT state
            if state == "INIT":
                candidates = []
                for track in tracks:
                    if not track.is_confirmed():
                        continue
                    try:
                        tid = track.track_id
                        x1, y1, x2, y2 = map(int, track.to_ltrb())
                        x1 = max(0, x1); y1 = max(0, y1)
                        x2 = min(frame.shape[1]-1, x2); y2 = min(frame.shape[0]-1, y2)
                        if x2 <= x1 or y2 <= y1:
                            continue
                        crop = frame[y1:y2, x1:x2]
                        fvec = self.extract_feature(crop)
                        if fvec is None:
                            continue
                        distance = self.estimate_distance((x1, y1, x2, y2))
                        cx = int((x1 + x2) / 2)
                        cy = int((y1 + y2) / 2)
                        candidates.append({
                            'tid': tid,
                            'fvec': fvec,
                            'bbox': (x1, y1, x2, y2),
                            'center': (cx, cy),
                            'distance': distance
                        })
                    except Exception:
                        continue

                if candidates:
                    closest = min(candidates, key=lambda x: x['distance'])
                    self.user_feature = closest['fvec']
                    self.active_id = closest['tid']
                    self.last_user_center = closest['center']
                    self.last_seen = current_time
                    state = "ACTIVE"
                    print(f"[INIT] Registered user ID: {self.active_id}")
                else:
                    # no candidate this cycle
                    pass

            # WAITING state: attempt to re-identify similarly to original
            if state == "WAITING":
                candidates = []
                for track in tracks:
                    if not track.is_confirmed():
                        continue
                    try:
                        tid = track.track_id
                        x1, y1, x2, y2 = map(int, track.to_ltrb())
                        x1 = max(0, x1); y1 = max(0, y1)
                        x2 = min(frame.shape[1]-1, x2); y2 = min(frame.shape[0]-1, y2)
                        if x2 <= x1 or y2 <= y1:
                            continue
                        crop = frame[y1:y2, x1:x2]
                        fvec = self.extract_feature(crop)
                        if fvec is None:
                            continue
                        distance = self.estimate_distance((x1, y1, x2, y2))
                        candidates.append({
                            'tid': tid,
                            'bbox': (x1, y1, x2, y2),
                            'center': (int((x1+x2)/2), int((y1+y2)/2)),
                            'distance': distance,
                            'fvec': fvec
                        })
                    except Exception:
                        continue

                nearby_candidates = [c for c in candidates if c['distance'] < 3.0]
                best_match = None
                min_similarity = 1.0
                for cand in nearby_candidates:
                    if self.user_feature is None or cand['fvec'] is None:
                        continue
                    similarity = cosine(cand['fvec'], self.user_feature)
                    if similarity < min_similarity:
                        min_similarity = similarity
                        best_match = {**cand, 'similarity': similarity}

                if best_match and best_match['similarity'] < self.COSINE_THRESHOLD:
                    self.active_id = best_match['tid']
                    self.last_seen = current_time
                    self.last_user_center = best_match['center']
                    state = "ACTIVE"
                    found_user = True
                    print(f"[RECOVER] User re-identified: ID={self.active_id}, sim={best_match['similarity']:.4f}")

            if state == "ACTIVE" and not found_user:
                if current_time - self.last_seen > self.LOST_TIMEOUT:
                    state = "WAITING"
                    print("[WAITING] User lost → entering WAITING state")
                    send_queue.put({
                                "center_point": [],
                                "distance": float(1),
                            })

            if state == "WAITING" and self.is_obstacle_ahead():
                print("[LIDAR] Obstacle ahead → possible user re-approaching")

            # Visualization: send latest frame and boxes without blocking producer
            try:
                viz_item = {"frame": frame.copy(), "bboxes": bboxes_for_viz, "state": state}
                try:
                    visualization_queue.put_nowait(viz_item)
                except Exception:
                    # drop oldest viz item and try once
                    try:
                        visualization_queue.get_nowait()
                    except Exception:
                        pass
                    try:
                        visualization_queue.put_nowait(viz_item)
                    except Exception:
                        pass
            except Exception:
                pass

            # occasional debug print
            if time.time() - last_print > 5:
                try:
                    qsize = frame_queue.qsize()
                except Exception:
                    qsize = -1
                print(f"[QUEUE] frame_queue size (approx): {qsize}  tracks: {len(tracks)}  state: {state}")
                last_print = time.time()

    @staticmethod
    def visualization_worker(visualization_queue):
        while True:
            try:
                data = visualization_queue.get(timeout=1)
            except Exception:
                continue

            frame = data["frame"]
            bboxes = data["bboxes"]
            state = data["state"]

            for bbox in bboxes:
                x1, y1, x2, y2 = bbox["x1"], bbox["y1"], bbox["x2"], bbox["y2"]
                tid = bbox["tid"]
                dist = bbox["distance"]
                color = bbox["color"]

                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                label = f"ID: {tid}  Dist: {dist:.1f}m"
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            cv2.putText(frame, f"State: {state}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow("Tracking", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break

        cv2.destroyAllWindows()

def run_tracking_processor(frame_queue, send_queue, visualization_queue):
    tracking_processor = Yolo_Deepsort()
    tracking_processor.main_processor(frame_queue, send_queue, visualization_queue)

if __name__ == "__main__":
    multiprocessing.set_start_method('spawn', force=True)
    print("현재 start method:", multiprocessing.get_start_method())

    # Bounded queues to avoid unlimited memory growth
    frame_queue = multiprocessing.Queue(maxsize=FRAME_QUEUE_MAXSIZE)
    send_queue = multiprocessing.Queue(maxsize=1)
    visualization_queue = multiprocessing.Queue(maxsize=4)

    p1 = multiprocessing.Process(target=udp_video_receiver, args=(frame_queue,), daemon=True)
    p2 = multiprocessing.Process(target=run_tracking_processor, args=(frame_queue, send_queue, visualization_queue), daemon=True)
    p3 = multiprocessing.Process(target=Yolo_Deepsort.send_loop, args=(send_queue,), daemon=True)
    p4 = multiprocessing.Process(target=Yolo_Deepsort.visualization_worker, args=(visualization_queue,), daemon=True)

    p1.start()
    p2.start()
    p3.start()
    p4.start()

    print("[VISION] All processes started.")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("[VISION] Shutting down...")
