import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
#update시 들어온 내용 출력할것
class ArcsCurrentPositionPublisher(Node):
    def __init__(self):
        super().__init__('arcs_current_position_publisher')  # 노드 이름

        # current_position 토픽 발행
        self.current_position_pub = self.create_publisher(Point, '/current_position', 10)

        # amcl_pose 구독
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )

        self.get_logger().info("🚀 current_position_publisher started")

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        # PoseWithCovarianceStamped → Point 변환
        point_msg = Point()
        point_msg.x = msg.pose.pose.position.x
        point_msg.y = msg.pose.pose.position.y
        point_msg.z = msg.pose.pose.position.z  # 일반적으로 0.0

        # 발행
        self.current_position_pub.publish(point_msg)
        self.get_logger().info(
            f"📍 Current Position: x={point_msg.x:.2f}, y={point_msg.y:.2f}, z={point_msg.z:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ArcsCurrentPositionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
