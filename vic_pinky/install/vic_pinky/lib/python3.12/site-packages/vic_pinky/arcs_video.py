import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from ultralytics import YOLO
import cv2

class Yolovision(Node):
    def __init__(self):
        super().__init__('yolo_vision')

        self.vision_pub = self.create_publisher(Float32MultiArray, '/vision_data', 10)
        self.mode_pub = self.create_publisher(String, '/arcs_mode', 10)

        # 기본 모드 FOLLOWING 설정
        mode_msg = String()
        mode_msg.data = "FOLLOWING"
        self.mode_pub.publish(mode_msg)
        self.get_logger().info("🚦 Mode set to FOLLOWING")

        # YOLO 모델 로드
        self.model = YOLO('yolov8n.pt')

        # 카메라 설정
        self.cap = cv2.VideoCapture(0)
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f"📷 Camera resolution: {self.frame_width} x {self.frame_height}")

        # 화면 세로가 꽉 찼을 때 거리 1m
        self.REFERENCE_HEIGHT = 480  # px
        self.REFERENCE_DISTANCE = 1.0  # m

        self.create_timer(0.1, self.process_frame)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("⚠ Camera frame not received")
            return

        results = self.model(frame, verbose=False)
        person_detections = []

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                if self.model.names[cls_id] == 'person':
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    conf = float(box.conf[0])
                    person_detections.append((conf, (x1, y1, x2, y2)))

        if person_detections:
            _, (x1, y1, x2, y2) = max(person_detections, key=lambda x: x[0])

            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            bbox_height = int(y2 - y1)

            # 거리 계산 (높이 비율 기반)
            distance_m = (self.REFERENCE_HEIGHT / bbox_height) * self.REFERENCE_DISTANCE

            # 디스플레이용 시각화
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.circle(frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
            text = f"cx={center_x:.1f}, cy={center_y:.1f}, dist={distance_m:.2f}m"
            cv2.putText(frame, text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

            # ROS 메시지 발행
            msg = Float32MultiArray()
            msg.data = [float(center_x), float(center_y), float(distance_m)]
            self.vision_pub.publish(msg)
        else:
            # 사람이 없으면 빈 배열 전송
            self.get_logger().info("🚫 No person detected")
            msg = Float32MultiArray()
            msg.data = []
            self.vision_pub.publish(msg)

        cv2.imshow("YOLO Person Detection", frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = Yolovision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
