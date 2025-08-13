import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class ScanQuarterViewer(Node):
    def __init__(self):
        super().__init__('scan_quarter_viewer')
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.printed_once = False

    def scan_callback(self, msg):
        if self.printed_once:
            return

        ranges = list(msg.ranges)
        total_points = len(ranges)  # 보통 720개
        quarter = total_points // 4

        for i in range(4):
            start_idx = i * quarter
            end_idx = (i + 1) * quarter
            sector_data = ranges[start_idx:end_idx]
            self.get_logger().info(f"angle_min={msg.angle_min}, angle_max={msg.angle_max}")

            # self.get_logger().info(f"\n==== 구간 {i+1} (index {start_idx} ~ {end_idx-1}) ====")
            # for idx, dist in enumerate(sector_data, start=start_idx):
            #     if not math.isinf(dist) and not math.isnan(dist):
            #         self.get_logger().info(f"idx {idx}: {dist:.3f} m")

        self.printed_once = True

def main():
    rclpy.init()
    node = ScanQuarterViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
