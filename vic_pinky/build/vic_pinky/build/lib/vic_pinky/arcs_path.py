import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import Path

class PathThroughPosesTester(Node):
    def __init__(self):
        super().__init__('path_through_poses_tester')

        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        self.path_pub = self.create_publisher(Path, '/dummy_path', 10)

        # 1. 시작 pose
        start_pose = self.create_pose(5.5, 0.0, 0.0)

        # 2. 목적지 pose 리스트
        goal_poses = [
            self.create_pose(3.0, 0.0, 0.0),
            self.create_pose(1.0, 0.0, 0.0)
        ]

        # 3. 경로 생성
        path = self.navigator.getPathThroughPoses(start_pose, goal_poses)

        if path and len(path.poses) > 0:
            self.get_logger().info(f"✅ 경로 생성 성공: {len(path.poses)} 포즈 포함")
            self.path_pub.publish(path)
        else:
            self.get_logger().warn("❌ 경로 생성 실패 또는 비어 있음")

    def create_pose(self, x, y, yaw_deg):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # yaw = 0도 → 쿼터니언(0, 0, 0, 1)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        return pose

def main(args=None):
    rclpy.init(args=args)
    node = PathThroughPosesTester()
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
