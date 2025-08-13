import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math
import queue
import time
import numpy as np

class ArcsFollower(Node):
    def __init__(self):
        super().__init__('arcs_follower')
        self.get_logger().info("🚀 ArcsFollower 시작")

        # =========================
        # 기존 회피 관련 설정
        # =========================
        self.pillar_pub = self.create_publisher(Marker, '/pillar_box', 10)
        self.front_box_pub = self.create_publisher(Marker, '/front_box', 10)
        self.back_box_pub = self.create_publisher(Marker, '/back_box', 10)
        self.left_box_pub = self.create_publisher(Marker, '/left_box', 10)
        self.right_box_pub = self.create_publisher(Marker, '/right_box', 10)

        self.front_points_pub = self.create_publisher(Marker, '/front_obstacle_points', 10)
        self.back_points_pub = self.create_publisher(Marker, '/back_obstacle_points', 10)
        self.left_points_pub = self.create_publisher(Marker, '/left_obstacle_points', 10)
        self.right_points_pub = self.create_publisher(Marker, '/right_obstacle_points', 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.create_timer(0.1, self.main_loop)

        self.lidar_height = 0.25
        self.depth = 0.20
        self.pillars = [
            ((114 + 136) // 2, 0.2),
            ((317 + 324) // 2, 0.5),
            ((406 + 411) // 2, 0.5),
            ((593 + 614) // 2, 0.2)
        ]
        self.pillar_ignore_ranges = [
            (114 - 10, 136 + 10),
            (317 - 10, 324 + 10),
            (406 - 10, 411 + 10),
            (593 - 10, 614 + 10)
        ]

        self.front_obstacle_points = []
        self.back_obstacle_points = []
        self.left_obstacle_points = []
        self.right_obstacle_points = []

        self.is_executing = False
        self.current_direction = None
        self.command_queue = queue.Queue()
        self.angle_increment = None

        # =========================
        # 사람 추종 관련 설정
        # =========================
        self.create_subscription(String, '/arcs_mode', self.mode_callback, 10)
        self.create_subscription(Point, '/vision_data', self.vision_callback, 10)

        self.state = "WAITING"
        self.prev_state = None
        self.center_x = 160.0
        self.target_distance = 1.0
        self.vision_x = None
        self.vision_z = None
        self.last_time = None

        # PID 계수 + Deadzone
        self.rotation_kp, self.rotation_kd = 0.5, 0.02
        self.forward_kp, self.forward_kd = 0.5, 0.02
        self.prev_rot_error = 0.0
        self.prev_fwd_error = 0.0
        self.deadzone_x = 50
        self.deadzone_z = 0.2


    def mode_callback(self, msg):
        new_state = msg.data.strip().upper()
        prev = self.state
        if new_state == prev:
            return  # 동일 상태 반복 발행이면 무시

        # FOLLOWING → 비-FOLLOWING으로 바뀔 때만 진짜 긴급정지
        if prev == "FOLLOWING" and new_state != "FOLLOWING":
            self.emergency_stop()
        else:
            # 그 외 전이에서는 소프트 스톱만 (로그 없음)
            self.stop_robot_follow()

        self.prev_state = prev
        self.state = new_state

    def vision_callback(self, msg):
        self.vision_x = msg.x
        self.vision_z = msg.z

        if self.state == "FOLLOWING":
            if self.vision_x is not None and self.vision_z is not None:
                self.get_logger().info(f"사람 위치: x={self.vision_x}, z={self.vision_z}")
            else:
                self.get_logger().warn("사람 위치 정보 없음")

    # =========================
    # 사람 추종
    # =========================
    def follow_person(self):
        if self.vision_x is None or self.vision_z is None:
            self.stop_robot_follow()
            return
        
        x_error = self.vision_x - self.center_x
        z_error = self.vision_z - self.target_distance

        if abs(x_error) < self.deadzone_x:
            x_error = 0.0
        if abs(z_error) < self.deadzone_z:
            z_error = 0.0

        now = time.time()
        dt = now - self.last_time if self.last_time else 0.1
        self.last_time = now

        rot_deriv = (x_error - self.prev_rot_error) / dt
        fwd_deriv = (z_error - self.prev_fwd_error) / dt
        self.prev_rot_error = x_error
        self.prev_fwd_error = z_error

        angular_z = - (self.rotation_kp * x_error + self.rotation_kd * rot_deriv)
        linear_x = self.forward_kp * z_error + self.forward_kd * fwd_deriv

        angular_z = max(min(angular_z, 0.5), -0.5)
        linear_x = max(min(linear_x, 0.5), -0.5)

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("사람 추종중")
    # =========================
    # 메인 루프
    # =========================
    def main_loop(self):
        if self.state == "FOLLOWING":
            # 회피 중이면 추종 중지
            if self.is_executing:
                return

            self.follow_person()
    # ===== 큐 관리 =====
    def enqueue_command(self, func, *args, **kwargs):
        self.command_queue.put((func, args, kwargs))
        self.try_execute_next()

    def try_execute_next(self):
        if self.is_executing:
            return
        if not self.command_queue.empty():
            func, args, kwargs = self.command_queue.get()
            self.is_executing = True
            func(*args, **kwargs)

    def finish_command(self):
        self.is_executing = False
        self.current_direction = None
        self.try_execute_next()

    # ===== 안전 검사 =====
    def safe_to_move_forward(self):
        return not self.front_obstacle_points
    
    def safe_to_move_backward(self):
        return not self.back_obstacle_points
    
    def safe_to_turn_left(self):
        # 좌회전하려면 오른쪽이 비어 있어야 함
        return not self.right_obstacle_points

    def safe_to_turn_right(self):
        # 우회전하려면 왼쪽이 비어 있어야 함
        return not self.left_obstacle_points

    def emergency_stop(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

        # 타이머 제거
        if hasattr(self, 'stop_timer'):
            self.destroy_timer(self.stop_timer)
            del self.stop_timer
        if hasattr(self, 'hold_timer'):
            self.destroy_timer(self.hold_timer)
            del self.hold_timer

        # 큐 비우기
        with self.command_queue.mutex:
            self.command_queue.queue.clear()

        self.is_executing = False
        self.current_direction = None
        self.get_logger().warn("🚨 긴급 정지: 회피 동작 중단")



    # ===== 이동/회전 동작 =====
    def move_backward(self, duration=1.0, speed=-0.1):
        self.get_logger().info("후진 시작")
        self.current_direction = "backward"
        if not self.safe_to_move_backward():
            self.get_logger().warn("🚫 후진 경로에 장애물 → 실행 취소")
            self.emergency_stop()
            return
        twist = Twist()
        twist.linear.x = speed
        self.cmd_vel_pub.publish(twist)
        self.stop_timer = self.create_timer(duration, self.stop_robot_avoid)

    def move_forward(self, duration=1.0, speed=0.1):
        self.get_logger().info("전진 시작")
        self.current_direction = "forward"
        if not self.safe_to_move_forward():
            self.get_logger().warn("🚫 전진 경로에 장애물 → 실행 취소")
            self.emergency_stop()
            return
        twist = Twist()
        twist.linear.x = speed
        self.cmd_vel_pub.publish(twist)
        self.stop_timer = self.create_timer(duration, self.stop_robot_avoid)

    def turn_left(self, duration=0.5, speed=0.5):
        self.get_logger().info("좌회전 시작")
        self.current_direction = "left"
        if not self.safe_to_turn_left():
            self.get_logger().warn("🚫 우측에 장애물 → 좌회전 불가")
            self.emergency_stop()
            return
        twist = Twist()
        twist.angular.z = speed
        self.cmd_vel_pub.publish(twist)
        self.stop_timer = self.create_timer(duration, self.stop_robot_avoid)

    def turn_right(self, duration=0.5, speed=-0.5):
        self.get_logger().info("우회전 시작")
        self.current_direction = "right"
        if not self.safe_to_turn_right():
            self.get_logger().warn("🚫 좌측에 장애물 → 우회전 불가")
            self.emergency_stop()
            return
        twist = Twist()
        twist.angular.z = speed
        self.cmd_vel_pub.publish(twist)
        self.stop_timer = self.create_timer(duration, self.stop_robot_avoid)

    def stop_robot_follow(self):
        # 회피 중이면 추종 멈춤 실행하지 않음
        if self.is_executing:
            return
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def stop_robot_avoid(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        if hasattr(self, 'stop_timer'):
            self.destroy_timer(self.stop_timer)
            del self.stop_timer
        self.hold_timer = self.create_timer(1.0, self.stop_hold_done)
        
    def stop_hold_done(self):
        if hasattr(self, 'hold_timer'):
            self.destroy_timer(self.hold_timer)
            del self.hold_timer
        self.finish_command()

    # ===== 라이다 처리 =====
    def index_to_xy(self, index, distance):
        angle_rad = index * self.angle_increment
        x = distance * math.cos(angle_rad)
        y = distance * math.sin(angle_rad)
        return (x, y)

    def make_point(self, x, y, z=0.0):
        p = Point()
        p.x, p.y, p.z = x, y, z
        return p

    def is_pillar_index(self, index):
        for start, end in self.pillar_ignore_ranges:
            if start <= index <= end:
                return True
        return False

    def point_in_rect(self, px, py, rect):
        def cross(o, a, b):
            return (a[0]-o[0]) * (b[1]-o[1]) - (a[1]-o[1]) * (b[0]-o[0])
        sign = None
        for i in range(len(rect)):
            a = rect[i]
            b = rect[(i+1) % len(rect)]
            c = cross(a, (px, py), b)
            if sign is None:
                sign = c > 0
            elif (c > 0) != sign:
                return False
        return True

    # ===== 박스 좌표 계산 함수 (전/후/좌/우) =====
    def get_front_box_points(self):
        p1 = self.index_to_xy(*self.pillars[0])
        p2 = self.index_to_xy(*self.pillars[3])
        vec = (p2[0] - p1[0], p2[1] - p1[1])
        width_len = math.sqrt(vec[0]**2 + vec[1]**2)
        dir_width = (vec[0] / width_len, vec[1] / width_len)
        dir_depth = (-dir_width[1], dir_width[0])
        p3 = (p2[0] + self.depth * dir_depth[0], p2[1] + self.depth * dir_depth[1])
        p4 = (p1[0] + self.depth * dir_depth[0], p1[1] + self.depth * dir_depth[1])
        return [p1, p2, p3, p4]

    def get_back_box_points(self):
        p1 = self.index_to_xy(*self.pillars[1])
        p2 = self.index_to_xy(*self.pillars[2])
        vec = (p2[0] - p1[0], p2[1] - p1[1])
        width_len = math.sqrt(vec[0]**2 + vec[1]**2)
        dir_width = (vec[0] / width_len, vec[1] / width_len)
        dir_depth = (-dir_width[1], dir_width[0])
        p3 = (p1[0] - self.depth * dir_depth[0], p1[1] - self.depth * dir_depth[1])
        p4 = (p2[0] - self.depth * dir_depth[0], p2[1] - self.depth * dir_depth[1])
        return [p1, p2, p4, p3]

    def get_left_box_points(self):
        p_front_left = self.index_to_xy(*self.pillars[0])
        p_back_left = self.index_to_xy(*self.pillars[1])
        vec_fb = (p_back_left[0] - p_front_left[0], p_back_left[1] - p_front_left[1])
        length_fb = math.sqrt(vec_fb[0] ** 2 + vec_fb[1] ** 2)
        dir_fb = (vec_fb[0] / length_fb, vec_fb[1] / length_fb)
        dir_out = (dir_fb[1], -dir_fb[0])
        p_front_extended = (
            p_front_left[0] - self.depth * dir_fb[0],
            p_front_left[1] - self.depth * dir_fb[1]
        )
        p_back_extended = (
            p_back_left[0] + self.depth * dir_fb[0],
            p_back_left[1] + self.depth * dir_fb[1]
        )
        p1 = (p_front_extended[0] + self.depth * dir_out[0],
              p_front_extended[1] + self.depth * dir_out[1])
        p2 = (p_back_extended[0] + self.depth * dir_out[0],
              p_back_extended[1] + self.depth * dir_out[1])
        p3 = (p_back_extended[0], p_back_extended[1])
        p4 = (p_front_extended[0], p_front_extended[1])
        return [p1, p2, p3, p4]

    def get_right_box_points(self):
        p_front_right = self.index_to_xy(*self.pillars[3])
        p_back_right = self.index_to_xy(*self.pillars[2])
        vec_fb = (p_back_right[0] - p_front_right[0], p_back_right[1] - p_front_right[1])
        length_fb = math.sqrt(vec_fb[0] ** 2 + vec_fb[1] ** 2)
        dir_fb = (vec_fb[0] / length_fb, vec_fb[1] / length_fb)
        dir_out = (-dir_fb[1], dir_fb[0])
        p_front_extended = (
            p_front_right[0] - self.depth * dir_fb[0],
            p_front_right[1] - self.depth * dir_fb[1]
        )
        p_back_extended = (
            p_back_right[0] + self.depth * dir_fb[0],
            p_back_right[1] + self.depth * dir_fb[1]
        )
        p1 = (p_front_extended[0] + self.depth * dir_out[0],
              p_front_extended[1] + self.depth * dir_out[1])
        p2 = (p_back_extended[0] + self.depth * dir_out[0],
              p_back_extended[1] + self.depth * dir_out[1])
        p3 = (p_back_extended[0], p_back_extended[1])
        p4 = (p_front_extended[0], p_front_extended[1])
        return [p1, p2, p3, p4]

    # ===== 스캔 처리 =====
    def scan_callback(self, msg: LaserScan):
        self.front_obstacle_points.clear()
        self.back_obstacle_points.clear()
        self.left_obstacle_points.clear()
        self.right_obstacle_points.clear()

        self.angle_increment = msg.angle_increment

        front_rect = self.get_front_box_points()
        back_rect  = self.get_back_box_points()
        left_rect  = self.get_left_box_points()
        right_rect = self.get_right_box_points()

        for i, r in enumerate(msg.ranges):
            if self.is_pillar_index(i): continue
            if math.isinf(r) or math.isnan(r): continue
            angle = i * msg.angle_increment
            x = r * math.cos(angle); y = r * math.sin(angle)
            if self.point_in_rect(x, y, front_rect): self.front_obstacle_points.append((x, y))
            if self.point_in_rect(x, y, back_rect):  self.back_obstacle_points.append((x, y))
            if self.point_in_rect(x, y, left_rect):  self.left_obstacle_points.append((x, y))
            if self.point_in_rect(x, y, right_rect): self.right_obstacle_points.append((x, y))


        self.publish_obstacle_points()
        self.publish_boxes()

        if self.state == "FOLLOWING":
            # === 사람 추종 중 장애물 감지 ===
            if self.is_executing and self.current_direction:
                if self.current_direction == "forward" and self.front_obstacle_points:
                    self.get_logger().warn("🚨 전진 중 장애물 감지 → 긴급정지")
                    self.emergency_stop()
                    return
                elif self.current_direction == "backward" and self.back_obstacle_points:
                    self.get_logger().warn("🚨 후진 중 장애물 감지 → 긴급정지")
                    self.emergency_stop()
                    return
                elif self.current_direction == "left" and self.right_obstacle_points:
                    self.get_logger().warn("🚨 좌회전 중(우측) 장애물 감지 → 긴급정지")
                    self.emergency_stop()
                    return
                elif self.current_direction == "right" and self.left_obstacle_points:
                    self.get_logger().warn("🚨 우회전 중(좌측) 장애물 감지 → 긴급정지")
                    self.emergency_stop()
                    return
            # === 기존 회피 큐 === (그대로)
            if not self.is_executing:
                if self.front_obstacle_points:
                    self.enqueue_command(self.move_backward, 3.0, -0.1)
                if self.back_obstacle_points:
                    self.enqueue_command(self.move_forward, 3.0, 0.1)
                if self.left_obstacle_points:
                    self.enqueue_command(self.turn_left, 1.5, 0.5)
                    self.enqueue_command(self.move_backward, 3.0, -0.1)
                    self.enqueue_command(self.turn_right, 1.5, -0.5)
                    self.enqueue_command(self.move_forward, 3.0, 0.1)
                if self.right_obstacle_points:
                    self.enqueue_command(self.turn_right, 1.5, -0.5)
                    self.enqueue_command(self.move_backward, 3.0, -0.1)
                    self.enqueue_command(self.turn_left, 1.5, 0.5)
                    self.enqueue_command(self.move_forward, 3.0, 0.1)


    # ===== 마커 표시 =====
    def publish_obstacle_points(self):
        def make_marker(ns, points, pub, r, g, b):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = ns
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 1.0
            for x, y in points:
                marker.points.append(self.make_point(x, y, self.lidar_height))
            pub.publish(marker)

        make_marker("front_obstacles", self.front_obstacle_points, self.front_points_pub, 1.0, 1.0, 0.0)
        make_marker("back_obstacles", self.back_obstacle_points, self.back_points_pub, 0.0, 1.0, 1.0)
        make_marker("left_obstacles", self.left_obstacle_points, self.left_points_pub, 0.0, 1.0, 0.0)
        make_marker("right_obstacles", self.right_obstacle_points, self.right_points_pub, 1.0, 0.0, 0.0)


    def publish_box(self, rect_points, pub, color):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.a = 1.0
        marker.color.r, marker.color.g, marker.color.b = color
        top_points = [(x, y, self.lidar_height) for x, y in rect_points]
        bottom_points = [(x, y, 0.0) for x, y in rect_points]
        for i in range(4):
            marker.points.append(self.make_point(*top_points[i]))
            marker.points.append(self.make_point(*top_points[(i + 1) % 4]))
            marker.points.append(self.make_point(*bottom_points[i]))
            marker.points.append(self.make_point(*bottom_points[(i + 1) % 4]))
            marker.points.append(self.make_point(*bottom_points[i]))
            marker.points.append(self.make_point(*top_points[i]))
        pub.publish(marker)

    def publish_boxes(self):
        if self.angle_increment is None:
            return
        self.publish_box(self.get_front_box_points(), self.front_box_pub, (0.0, 0.0, 1.0))
        self.publish_box(self.get_back_box_points(),  self.back_box_pub,  (1.0, 0.0, 0.0))
        self.publish_box(self.get_left_box_points(),  self.left_box_pub,  (0.0, 1.0, 0.0))
        self.publish_box(self.get_right_box_points(), self.right_box_pub, (1.0, 0.0, 1.0))



def main(args=None):
    rclpy.init(args=args)
    node = ArcsFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
