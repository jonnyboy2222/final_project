import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import json  # JSON 변환용

class ArcsArrivalSender(Node):
    def __init__(self):
        super().__init__('arcs_arrival_sender')

        # UDP 설정
        self.udp_ip = "192.168.0.145"  # Main Service IP
        self.udp_port = 54321
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # /destination_arrived 구독
        self.create_subscription(String, '/destination_arrived', self.arrived_callback, 10)

        self.get_logger().info("🚀 ArcsArrivalSender started")

    def arrived_callback(self, msg: String):
        if msg.data.strip().upper() == "ARRIVED":
            
            # JSON payload 생성 (공백 포함 기본 형태)
            payload_dict = {"robot_id": 1}
            payload_str = json.dumps(payload_dict)  # 공백 포함 {"robot_id": 1}
            payload_bytes = payload_str.encode('utf-8')

            # AR 패킷 생성 (header + length + command + payload)
            header = b'\x00'              # 1 byte
            length = len(payload_bytes).to_bytes(4, 'big')  # 4 bytes
            command = b'AR'               # 2 bytes
            packet = header + length + command + payload_bytes

            # UDP 전송
            try:
                self.sock.sendto(packet, (self.udp_ip, self.udp_port))
                self.get_logger().info(f"📡 Sent AR packet to {self.udp_ip}:{self.udp_port}")
                self.get_logger().info(f"📡 Sent UDP | command=AR | payload={payload_str}")
            except Exception as e:
                self.get_logger().error(f"❌ UDP send error: {e}")

def main():
    rclpy.init()
    node = ArcsArrivalSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
