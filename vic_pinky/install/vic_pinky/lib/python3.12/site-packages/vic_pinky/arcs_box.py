import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
import math


class FrontBackLeftBoxDetector(Node):
    def __init__(self):
        super().__init__('front_back_left_box_detector')

        # Publishers
        self.pillar_pub = self.create_publisher(Marker, '/pillar_box', 10)
        self.front_box_pub = self.create_publisher(Marker, '/front_box', 10)
        self.back_box_pub = self.create_publisher(Marker, '/back_box', 10)
        self.left_box_pub = self.create_publisher(Marker, '/left_box', 10)

        self.front_points_pub = self.create_publisher(Marker, '/front_obstacle_points', 10)
        self.back_points_pub = self.create_publisher(Marker, '/back_obstacle_points', 10)
        self.left_points_pub = self.create_publisher(Marker, '/left_obstacle_points', 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Timer
        self.timer = self.create_timer(0.5, self.publish_boxes)

        # 라이다 높이
        self.lidar_height = 0.25

        # 전면/후면/좌측 박스 깊이
        self.depth = 0.3

        # 기둥 정보 (index, distance)
        self.pillars = [
            ((114 + 136) // 2, 0.2),  # 좌측 전면
            ((317 + 324) // 2, 0.5),  # 좌측 후면
            ((406 + 411) // 2, 0.5),  # 우측 후면
            ((593 + 614) // 2, 0.2)   # 우측 전면
        ]

        # 기둥 인덱스 감지 제외 범위
        self.pillar_ignore_ranges = [
            (114 - 5, 136 + 5),  # 좌측 전면
            (317 - 5, 324 + 5),  # 좌측 후면
            (406 - 5, 411 + 5),  # 우측 후면
            (593 - 5, 614 + 5)   # 우측 전면
        ]

        # 장애물 포인트 저장
        self.front_obstacle_points = []
        self.back_obstacle_points = []
        self.left_obstacle_points = []

        # 주행 상태 플래그
        self.is_backing = False
        self.is_forwarding = False
        self.is_turning = False

    # ---------------------------
    # 좌표 변환 / 점 생성
    # ---------------------------
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

    # ---------------------------
    # 박스 좌표 계산
    # ---------------------------
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
        # 좌측 전면 & 좌측 후면 기둥 좌표
        p_front_left = self.index_to_xy(*self.pillars[0])
        p_back_left = self.index_to_xy(*self.pillars[1])

        # 기둥을 잇는 벡터 (전면→후면)
        vec_fb = (p_back_left[0] - p_front_left[0], p_back_left[1] - p_front_left[1])
        length_fb = math.sqrt(vec_fb[0] ** 2 + vec_fb[1] ** 2)
        dir_fb = (vec_fb[0] / length_fb, vec_fb[1] / length_fb)  # 정규화

        # 좌측 바깥 방향 단위 벡터 (기둥선에 수직)
        dir_out = (dir_fb[1], -dir_fb[0])

        # depth 만큼 전면 방향 연장
        p_front_extended = (
            p_front_left[0] - self.depth * dir_fb[0],  # 전면이니까 반대 방향(-)
            p_front_left[1] - self.depth * dir_fb[1]
        )

        # depth 만큼 후면 방향 연장
        p_back_extended = (
            p_back_left[0] + self.depth * dir_fb[0],  # 후면이니까 같은 방향(+)
            p_back_left[1] + self.depth * dir_fb[1]
        )

        # 각 점을 바깥쪽으로 depth만큼 이동
        p1 = (p_front_extended[0] + self.depth * dir_out[0],
            p_front_extended[1] + self.depth * dir_out[1])
        p2 = (p_back_extended[0] + self.depth * dir_out[0],
            p_back_extended[1] + self.depth * dir_out[1])
        p3 = (p_back_extended[0], p_back_extended[1])
        p4 = (p_front_extended[0], p_front_extended[1])

        return [p1, p2, p3, p4]

    # ---------------------------
    # 동작 함수
    # ---------------------------
    def move_backward(self, duration=1.0, speed=-0.1):
        if self.is_backing or self.is_forwarding or self.is_turning:
            return
        self.is_backing = True
        twist = Twist()
        twist.linear.x = speed
        self.cmd_vel_pub.publish(twist)
        self.stop_timer = self.create_timer(duration, self.stop_robot)

    def move_forward(self, duration=1.0, speed=0.1):
        if self.is_forwarding or self.is_backing or self.is_turning:
            return
        self.is_forwarding = True
        twist = Twist()
        twist.linear.x = speed
        self.cmd_vel_pub.publish(twist)
        self.stop_timer = self.create_timer(duration, self.stop_robot)

    def turn_right(self, duration=1.0, speed=-0.5):
        if self.is_forwarding or self.is_backing or self.is_turning:
            return
        self.is_turning = True
        twist = Twist()
        twist.angular.z = speed
        self.cmd_vel_pub.publish(twist)
        self.stop_timer = self.create_timer(duration, self.stop_robot)

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        if self.is_backing:
            self.is_backing = False
        if self.is_forwarding:
            self.is_forwarding = False
        if self.is_turning:
            self.is_turning = False
        if hasattr(self, 'stop_timer'):
            self.destroy_timer(self.stop_timer)
            del self.stop_timer

    # ---------------------------
    # 라이다 콜백
    # ---------------------------
    def scan_callback(self, msg: LaserScan):
        self.front_obstacle_points.clear()
        self.back_obstacle_points.clear()
        self.left_obstacle_points.clear()
        self.angle_increment = msg.angle_increment

        front_rect = self.get_front_box_points()
        back_rect = self.get_back_box_points()
        left_rect = self.get_left_box_points()

        for i, r in enumerate(msg.ranges):
            if self.is_pillar_index(i):
                continue
            if math.isinf(r) or math.isnan(r):
                continue
            angle = i * msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            if self.point_in_rect(x, y, front_rect):
                self.front_obstacle_points.append((x, y))
            if self.point_in_rect(x, y, back_rect):
                self.back_obstacle_points.append((x, y))
            if self.point_in_rect(x, y, left_rect):
                self.left_obstacle_points.append((x, y))

        # 동작 로직
        if self.front_obstacle_points:
            self.move_backward()
        if self.back_obstacle_points:
            self.move_forward()
        if self.left_obstacle_points:
            self.move_backward(duration=1.0)
            self.turn_right(duration=1.0)
            

        self.publish_obstacle_points()

    # ---------------------------
    # 마커 표시
    # ---------------------------
    def publish_obstacle_points(self):
        def make_marker(ns, points, pub):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = ns
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.color.r = 1.0
            marker.color.a = 1.0
            for x, y in points:
                marker.points.append(self.make_point(x, y, self.lidar_height))
            pub.publish(marker)

        make_marker("front_obstacles", self.front_obstacle_points, self.front_points_pub)
        make_marker("back_obstacles", self.back_obstacle_points, self.back_points_pub)
        make_marker("left_obstacles", self.left_obstacle_points, self.left_points_pub)

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
        self.publish_box(self.get_front_box_points(), self.front_box_pub, (0.0, 0.0, 1.0))  # 파랑
        self.publish_box(self.get_back_box_points(), self.back_box_pub, (1.0, 0.0, 0.0))    # 빨강
        self.publish_box(self.get_left_box_points(), self.left_box_pub, (0.0, 1.0, 0.0))    # 초록


def main(args=None):
    rclpy.init(args=args)
    node = FrontBackLeftBoxDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
