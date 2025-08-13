#!/usr/bin/env python3
import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, TransformStamped

import tf2_ros


class PersonFollowerWithCostmap(Node):
    def __init__(self):
        super().__init__('person_follower_with_costmap')
        self.get_logger().info("🚀 PersonFollowerWithCostmap started")

        # ---------------- Parameters ----------------
        self.declare_parameter('vision_topic', '/vision_data')
        self.declare_parameter('mode_topic', '/arcs_mode')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('costmap_frame', 'odom')   # 보통 local_costmap frame
        self.declare_parameter('base_link_frame', 'base_link')

        self.declare_parameter('image_center_x', 320.0)
        self.declare_parameter('target_distance', 1.0)

        # Vision PID gains (속도/회전 완화)
        self.declare_parameter('k_heading', 0.003)  # 픽셀→각도 근사 계수 (작게)
        self.declare_parameter('k_dist',    0.35)   # 거리 에러→선속 (작게)
        self.declare_parameter('deadzone_px', 40.0)
        self.declare_parameter('deadzone_m', 0.15)

        # Limits (느리게)
        self.declare_parameter('max_lin', 0.25)
        self.declare_parameter('max_ang', 0.6)

        # Costmap lookahead & safety
        self.declare_parameter('lookahead_m', 1.2)
        self.declare_parameter('candidate_deg_span', 50)
        self.declare_parameter('candidate_deg_step', 5)
        self.declare_parameter('clearance_stop_th', 0.45)
        self.declare_parameter('clearance_slow_th', 0.8)

        # Scoring weights
        self.declare_parameter('w_vision', 1.0)
        self.declare_parameter('w_cost',   2.0)

        # Pull params
        self.vision_topic = self.get_parameter('vision_topic').get_parameter_value().string_value
        self.mode_topic   = self.get_parameter('mode_topic').get_parameter_value().string_value
        self.cmd_vel_topic= self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.costmap_topic= self.get_parameter('costmap_topic').get_parameter_value().string_value
        self.costmap_frame= self.get_parameter('costmap_frame').get_parameter_value().string_value
        self.base_link_frame = self.get_parameter('base_link_frame').get_parameter_value().string_value

        self.center_x = self.get_parameter('image_center_x').value
        self.target_distance = self.get_parameter('target_distance').value

        self.k_heading = self.get_parameter('k_heading').value
        self.k_dist    = self.get_parameter('k_dist').value
        self.dead_px   = self.get_parameter('deadzone_px').value
        self.dead_m    = self.get_parameter('deadzone_m').value

        self.max_lin   = self.get_parameter('max_lin').value
        self.max_ang   = self.get_parameter('max_ang').value

        self.lookahead = self.get_parameter('lookahead_m').value
        self.c_span    = self.get_parameter('candidate_deg_span').value
        self.c_step    = self.get_parameter('candidate_deg_step').value
        self.stop_th   = self.get_parameter('clearance_stop_th').value
        self.slow_th   = self.get_parameter('clearance_slow_th').value

        self.w_vision  = self.get_parameter('w_vision').value
        self.w_cost    = self.get_parameter('w_cost').value

        # ---------------- I/O ----------------
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Float32MultiArray, self.vision_topic, self.on_vision, 10)
        self.create_subscription(String, self.mode_topic, self.on_mode, 10)
        self.create_subscription(OccupancyGrid, self.costmap_topic, self.on_costmap, 10)

        # TF buffer for base_link in costmap frame
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------------- State ----------------
        self.state = "WAITING"   # WAITING / FOLLOWING
        self.vision_x = None
        self.vision_z = None
        self.costmap = None

        # 램핑(가속 제한) 상태
        self.prev_lin = 0.0
        self.prev_ang = 0.0
        self.ctrl_dt  = 0.1      # timer period
        self.max_lin_acc = 0.15  # m/s^2
        self.max_ang_acc = 0.8   # rad/s^2

        # Control timer
        self.timer = self.create_timer(self.ctrl_dt, self.control_loop)

    # ===================== Callbacks =====================
    def on_mode(self, msg: String):
        self.state = msg.data
        if self.state != "FOLLOWING":
            self.stop()

    def on_vision(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self.vision_x, _, self.vision_z = msg.data
        else:
            self.vision_x = self.vision_z = None

    def on_costmap(self, msg: OccupancyGrid):
        self.costmap = msg

    # ===================== Utils =====================
    def stop(self):
        tw = Twist()
        self.prev_lin = 0.0
        self.prev_ang = 0.0
        self.cmd_pub.publish(tw)

    def ramp(self, prev, target, step_limit):
        if target > prev + step_limit:
            return prev + step_limit
        if target < prev - step_limit:
            return prev - step_limit
        return target

    def get_robot_pose_in_costmap(self):
        """base_link pose in costmap frame (x,y,yaw)."""
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                self.costmap_frame, self.base_link_frame, rclpy.time.Time())
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            q = tf.transform.rotation
            siny_cosp = 2*(q.w*q.z + q.x*q.y)
            cosy_cosp = 1 - 2*(q.y*q.y + q.z*q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return True, (x, y, yaw)
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return False, (0.0, 0.0, 0.0)

    def world_to_grid(self, wx, wy):
        if self.costmap is None:
            return False, (0, 0)
        res = self.costmap.info.resolution
        ox  = self.costmap.info.origin.position.x
        oy  = self.costmap.info.origin.position.y
        col = int((wx - ox)/res)
        row = int((wy - oy)/res)
        if 0 <= col < self.costmap.info.width and 0 <= row < self.costmap.info.height:
            return True, (col, row)
        return False, (col, row)

    def get_cell(self, col, row):
        i = row * self.costmap.info.width + col
        return self.costmap.data[i]

    def ray_clearance(self, start_xy, theta_world, max_dist):
        """
        theta_world 방향으로 max_dist까지 전진하며
        첫 충돌 전까지의 거리(클리어런스)를 반환. 충돌 없으면 max_dist.
        """
        if self.costmap is None:
            return 0.0
        (sx, sy) = start_xy
        res = self.costmap.info.resolution
        step = res * 0.8
        dist = 0.0
        while dist <= max_dist:
            wx = sx + dist * math.cos(theta_world)
            wy = sy + dist * math.sin(theta_world)
            ok, (c, r) = self.world_to_grid(wx, wy)
            if not ok:
                return dist  # 맵 밖: 안전하지 않다고 간주
            occ = self.get_cell(c, r)
            if occ < 0:
                return dist  # unknown을 보수적으로 장애물 취급
            if occ >= 50:
                return dist  # 점유
            dist += step
        return max_dist

    # ===================== Control =====================
    def control_loop(self):
        if self.state != "FOLLOWING":
            self.stop()
            return

        # 비전 유효성
        if self.vision_x is None or self.vision_z is None:
            self.stop()
            return

        # 코스트맵/TF 없으면 기본 추종
        if self.costmap is None:
            self.follow_basic()
            return

        ok, (rx, ry, ryaw) = self.get_robot_pose_in_costmap()
        if not ok:
            self.follow_basic()
            return

        # ---- 원하는 조향각: "오른쪽 목표 → 우회전(angular.z < 0)" 되도록 부호 반전 ----
        x_err = self.vision_x - self.center_x
        if abs(x_err) < self.dead_px:
            x_err = 0.0
        desired_yaw_err = - self.k_heading * x_err  # ★ sign flipped
        desired_heading_world = ryaw + desired_yaw_err

        # ---- 후보 각도: desired 부근 ±c_span ----
        degs = np.arange(-self.c_span, self.c_span + 1, self.c_step)
        candidates = []
        for d in degs:
            th = desired_heading_world + math.radians(d)
            clearance = self.ray_clearance((rx, ry), th, self.lookahead)
            vision_cost = abs(math.radians(d))
            costmap_cost = max(0.0, (self.lookahead - clearance))
            score = self.w_vision * vision_cost + self.w_cost * costmap_cost
            candidates.append((score, th, clearance))

        candidates.sort(key=lambda x: x[0])
        best_score, best_heading_world, best_clearance = candidates[0]

        # ---- 선속: 거리 에러 기반 + 안전 감속 ----
        z_err = self.vision_z - self.target_distance
        if abs(z_err) < self.dead_m:
            z_err = 0.0
        lin = self.k_dist * z_err

        if best_clearance <= self.stop_th:
            lin = 0.0
        elif best_clearance <= self.slow_th:
            lin = min(lin, 0.15)

        lin = max(min(lin, self.max_lin), -self.max_lin)

        # ---- 각속: 현재 yaw → best_heading_world ----
        yaw_err = math.atan2(math.sin(best_heading_world - ryaw),
                             math.cos(best_heading_world - ryaw))
        ang = 1.0 * yaw_err  # P제어
        ang = max(min(ang, self.max_ang), -self.max_ang)

        # 막혔는데 회전이 너무 작으면 회전 보강
        if best_clearance < 0.3 and abs(ang) < 0.2:
            ang = 0.3 * (1 if yaw_err >= 0 else -1)

        # ---- 램핑(가속 제한) 적용 ----
        lin_step = self.max_lin_acc * self.ctrl_dt
        ang_step = self.max_ang_acc * self.ctrl_dt
        lin = self.ramp(self.prev_lin, lin, lin_step)
        ang = self.ramp(self.prev_ang, ang, ang_step)
        self.prev_lin, self.prev_ang = lin, ang

        # ---- Publish ----
        tw = Twist()
        tw.linear.x = lin
        tw.angular.z = ang
        self.cmd_pub.publish(tw)

    def follow_basic(self):
        """코스트맵 미사용 시, 비전만으로 단순 추종 (부호 반전 포함)."""
        if self.vision_x is None or self.vision_z is None:
            self.stop()
            return

        x_err = self.vision_x - self.center_x
        if abs(x_err) < self.dead_px:
            x_err = 0.0
        ang = - self.k_heading * x_err  # ★ sign flipped

        z_err = self.vision_z - self.target_distance
        if abs(z_err) < self.dead_m:
            z_err = 0.0
        lin = self.k_dist * z_err

        ang = max(min(ang, self.max_ang), -self.max_ang)
        lin = max(min(lin, self.max_lin), -self.max_lin)

        # 램핑
        lin_step = self.max_lin_acc * self.ctrl_dt
        ang_step = self.max_ang_acc * self.ctrl_dt
        lin = self.ramp(self.prev_lin, lin, lin_step)
        ang = self.ramp(self.prev_ang, ang, ang_step)
        self.prev_lin, self.prev_ang = lin, ang

        tw = Twist()
        tw.linear.x = lin
        tw.angular.z = ang
        self.cmd_pub.publish(tw)


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerWithCostmap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
