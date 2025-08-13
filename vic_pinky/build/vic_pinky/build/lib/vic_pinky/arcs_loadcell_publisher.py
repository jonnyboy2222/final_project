import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class ArcsLoadcellPublisher(Node):
    def __init__(self):
        super().__init__('arcs_loadcell_publisher')
        self.get_logger().info("🚀 HX711 Serial Loadcell Publisher Started")

        # 시리얼 설정
        self.SERIAL_PORT = '/dev/ttyACM0'  # 포트 확인 필요
        self.BAUDRATE = 57600
        try:
            self.ser = serial.Serial(self.SERIAL_PORT, self.BAUDRATE, timeout=1)
            self.get_logger().info(f"📡 Serial connected: {self.SERIAL_PORT}")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Serial connection failed: {e}")
            raise SystemExit

        # ROS2 퍼블리셔
        self.pub = self.create_publisher(Float32, '/loadcell', 10)

        # 주기적으로 시리얼 읽기 (0.1초마다)
        self.create_timer(0.3, self.read_and_publish)

    def read_and_publish(self):
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                return

            # 디버그 로그
            self.get_logger().info(f"[Serial] 수신 데이터: {line}")

            # 숫자 값만 발행 (필요 시 float 변환)
            try:
                weight_val = float(line)
                msg = Float32()
                msg.data = weight_val
                self.pub.publish(msg)
                self.get_logger().info(f"📦 Published weight: {weight_val:.3f}")
            except ValueError:
                # 숫자가 아닌 경우 무시
                pass

        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ArcsLoadcellPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
