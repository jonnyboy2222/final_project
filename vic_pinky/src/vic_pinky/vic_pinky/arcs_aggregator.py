import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Point, PoseArray
import socket
import json

class ArcsAggregator(Node):
    def __init__(self):
        super().__init__('arcs_aggregator')

        # 최신 데이터 저장 변수
        self.loadcell_value = None
        self.current_position = None
        self.distance_value = None
        self.path_data = None

        # ROS 구독자
        self.create_subscription(Float32, '/loadcell', self.loadcell_callback, 10)
        self.create_subscription(Point, '/current_position', self.current_position_callback, 10)
        self.create_subscription(PoseArray, '/path', self.path_callback, 10)

        # UDP 설정
        self.udp_ip = "192.168.0.145"  # Main Service IP
        self.udp_port = 54321
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # 주기적으로 데이터 전송 (1초마다)
        self.timer_period = 2.0
        self.create_timer(self.timer_period, self.send_udp_data)

        self.get_logger().info("🚀 arcs_aggregator started, collecting & sending data via UDP")

    # ===== ROS 콜백 =====
    def loadcell_callback(self, msg: Float32):
        self.loadcell_value = msg.data

    def current_position_callback(self, msg: Point):
        # (x, y) 튜플로 저장
        self.current_position = [msg.x, msg.y]

    def path_callback(self, msg: PoseArray):
        if not msg.poses:
            self.path_data = None
            self.get_logger().info("path = None으로 초기화됨")
            return
        print(f"Callback called with {len(msg.poses)} poses")
        # (x, y) 튜플 리스트로 저장
        self.path_data = [
            [pose.position.x, pose.position.y]
            for pose in msg.poses
        ]

        print(f"Path data updated: {self.path_data}")

    # ===== UDP 데이터 전송 =====
    def send_udp_data(self):
        # JSON 데이터 구성
        data_dict = {
            "robot_id": 1,
            "loadcell": self.loadcell_value,
            "current_position": self.current_position,
            "path": self.path_data
        }

        print(f"Sending data: {data_dict}")

        # JSON 직렬화
        json_data = json.dumps(data_dict, ensure_ascii=False).encode('utf-8')

        # 패킷 구성 (header + length + json)
        header = b'\x00'  # 1 byte
        length = len(json_data).to_bytes(4, 'big')  # 4 bytes
        command = 'MV' 
        command_bytes = command.encode('ascii')  # 2 bytes
        packet = header + length + command_bytes + json_data

        # UDP 전송
        try:
            self.udp_sock.sendto(packet, (self.udp_ip, self.udp_port))
            self.get_logger().info(f"📡 Sent UDP packet ({len(json_data)} bytes)")
        except Exception as e:
            self.get_logger().error(f"❌ UDP send error: {e}")

def main():
    rclpy.init()
    node = ArcsAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
