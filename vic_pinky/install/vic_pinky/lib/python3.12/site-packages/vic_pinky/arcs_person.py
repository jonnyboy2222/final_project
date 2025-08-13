import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String
import time


class ArcsFollower(Node):
    def __init__(self):
        super().__init__('arcs_follower')
        self.get_logger().info("🚀 ArcsFollower 노드 시작")

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.create_subscription(String, '/arcs_mode', self.mode_callback, 10)
        self.create_subscription(Float32MultiArray, '/vision_data', self.vision_callback, 10)

        # Timer (10Hz)
        self.create_timer(0.1, self.main_loop)

        # Following parameters
        self.state = "WAITING"
        self.center_x = 320.0  # 화면 중심 픽셀
        self.target_distance = 1.0  # 목표 거리 (m)

        self.vision_x = None
        self.vision_z = None
        self.last_time = None

        # PID 계수
        self.rotation_kp, self.rotation_kd = 0.002, 0.0002
        self.forward_kp, self.forward_kd = 0.5, 0.1

        # PID 이전 오차 저장
        self.prev_rot_error = 0.0
        self.prev_fwd_error = 0.0

        # Deadzone 설정
        self.deadzone_x = 50      # 픽셀 단위
        self.deadzone_z = 0.2    # m 단위

    # ===== Mode Callback =====
    def mode_callback(self, msg):
        self.state = msg.data
        if self.state != "FOLLOWING":
            self.stop_robot()

    # ===== Vision Callback =====
    def vision_callback(self, msg):
        if len(msg.data) >= 3:
            self.vision_x, _, self.vision_z = msg.data
        else:
            self.vision_x = self.vision_z = None

    # ===== Main Loop =====
    def main_loop(self):
        if self.state != "FOLLOWING":
            return
        self.follow_person()

    # ===== Person Following =====
    def follow_person(self):
        if self.vision_x is None or self.vision_z is None:
            self.stop_robot()
            return

        # 오차 계산
        x_error = self.vision_x - self.center_x
        z_error = self.vision_z - self.target_distance

        # Deadzone 적용 → 작은 오차 무시
        if abs(x_error) < self.deadzone_x:
            x_error = 0.0
        if abs(z_error) < self.deadzone_z:
            z_error = 0.0

        # PID 시간 계산
        now = time.time()
        dt = now - self.last_time if self.last_time else 0.1
        self.last_time = now

        # PID 미분항
        rot_deriv = (x_error - self.prev_rot_error) / dt
        fwd_deriv = (z_error - self.prev_fwd_error) / dt

        # 이전 오차 저장
        self.prev_rot_error = x_error
        self.prev_fwd_error = z_error

        # PID 제어값 계산
        angular_z = - (self.rotation_kp * x_error + self.rotation_kd * rot_deriv)
        linear_x = self.forward_kp * z_error + self.forward_kd * fwd_deriv

        # 속도 제한
        angular_z = max(min(angular_z, 0.5), -0.5)
        linear_x = max(min(linear_x, 0.5), -0.5)

        # Twist 메시지 발행
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

    # ===== Stop =====
    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ArcsFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
