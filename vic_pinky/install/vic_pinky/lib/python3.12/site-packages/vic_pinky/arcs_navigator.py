import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import math
import tf_transformations
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Twist
from nav2_msgs.srv import ClearEntireCostmap
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter
import time

class ArcsNavigator(Node):
    def __init__(self):
        super().__init__('arcs_navigator')
        self.get_logger().info("🚀 ArcsNavigator initializing...")
        self.clear_path_timer = None
        # Nav2 Navigator 초기화
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        # 상태 플래그
        self.map_data = None
        self.map_ready = False
        self.current_pose = None
        self.pose_ready = False
        self.global_costmap_data = None
        self.global_costmap_ready = False

        # 초기 위치 설정
        self.set_initial_pose()
        
        # QoS 설정
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        pose_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        self.global_clear_client = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.local_clear_client = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')

        # 구독
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, pose_qos)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.global_costmap_callback, 10)
        self.create_subscription(PoseArray, '/des_coor', self.des_coor_callback, 10)
        self.create_subscription(PoseArray, '/pre_des_coor', self.pre_des_coor_callback, 10)
        self.create_subscription(String, '/pause_command', self.pause_command_callback, 10)

        # 발행
        self.path_pub = self.create_publisher(PoseArray, '/path', 10)
        self.arrived_pub = self.create_publisher(String, '/destination_arrived', 10)

        # 주행 상태 관리
        self.task_running = False
        self.current_goal = None
        self.monitor_timer = None

        self.get_logger().info("✅ ArcsNavigator started.")
  

    def set_inflation_radius(self, radius=1.0):
        targets = [
            '/global_costmap/global_costmap',
            '/local_costmap/local_costmap'
        ]

        for node_name in targets:
            client = self.create_client(SetParametersAtomically, node_name + '/set_parameters_atomically')
            if not client.wait_for_service(timeout_sec=3.0):
                self.get_logger().warn(f"⚠️ {node_name}/set_parameters_atomically 서비스 없음")
                continue

            param = Parameter(
                name='inflation_layer.inflation_radius',
                value=radius,
            )
            request = SetParametersAtomically.Request()
            request.parameters = [param.to_parameter_msg()]

            future = client.call_async(request)

            def callback(fut):
                result = fut.result()
                if result and result.result.successful:
                    self.get_logger().info(f"✅ {node_name} inflation_radius 설정 완료: {radius}")
                else:
                    self.get_logger().error(f"❌ {node_name} inflation_radius 설정 실패")

            future.add_done_callback(callback)

    def clear_costmaps(self):
        if self.global_clear_client.service_is_ready():
            self.get_logger().info("🧹 글로벌 costmap clear 요청 중...")
            req = ClearEntireCostmap.Request()
            self.global_clear_client.call_async(req)
        else:
            self.get_logger().warn("⚠️ 글로벌 costmap clear 서비스 준비 안됨")

        if self.local_clear_client.service_is_ready():
            self.get_logger().info("🧹 로컬 costmap clear 요청 중...")
            req = ClearEntireCostmap.Request()
            self.local_clear_client.call_async(req)
        else:
            self.get_logger().warn("⚠️ 로컬 costmap clear 서비스 준비 안됨")
    # ----------------------
    # 맵 데이터 수신
    # ----------------------
    def map_callback(self, msg: OccupancyGrid):
        if not self.map_ready:
            self.get_logger().info("🗺 Map received, ready for planning.")
            self.map_ready = True
        self.map_data = msg

    # ----------------------
    # AMCL 위치 수신
    # ----------------------
    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = msg.pose.pose
        if not self.pose_ready:
            self.get_logger().info("📍 AMCL pose received, ready for planning.")
            self.pose_ready = True

            # ✅ 최초 AMCL 수신 후 costmap clear
            self.clear_costmaps()
            self.set_inflation_radius()

        self.current_pose = pose_stamped

    # ----------------------
    # Global costmap 수신
    # ----------------------
    def global_costmap_callback(self, msg: OccupancyGrid):
        if not self.global_costmap_ready:
            self.get_logger().info("🌍 Global costmap received, ready for planning.")
            self.global_costmap_ready = True
        self.global_costmap_data = msg

    # ----------------------
    # /global_costmap 기준 free space 확인
    # ----------------------
    def is_free_space_costmap(self, x, y):
        if self.global_costmap_data is None:
            return None
        return self._check_free_space(self.global_costmap_data, x, y)

    # ----------------------
    # 공통 free space 판정 로직
    # ----------------------
    def _check_free_space(self, grid_msg, x, y):
        res = grid_msg.info.resolution
        origin_x = grid_msg.info.origin.position.x
        origin_y = grid_msg.info.origin.position.y
        width = grid_msg.info.width
        height = grid_msg.info.height

        mx = int((x - origin_x) / res)
        my = int((y - origin_y) / res)

        if mx < 0 or my < 0 or mx >= width or my >= height:
            return None

        occ_value = grid_msg.data[my * width + mx]
        return occ_value == 0  # 0이면 free space

    # ----------------------
    # 가장 가까운 free space 찾기
    # ----------------------
    def find_nearest_free_space(self, x, y, search_radius=1.0, step=0.05):
        """
        global_costmap에서 (x, y)와 가장 가까운 free space 좌표 반환
        """
        if self.global_costmap_data is None:
            return x, y

        res = self.global_costmap_data.info.resolution
        origin_x = self.global_costmap_data.info.origin.position.x
        origin_y = self.global_costmap_data.info.origin.position.y
        width = self.global_costmap_data.info.width
        height = self.global_costmap_data.info.height

        # 현재 좌표가 free space면 그대로 반환
        if self.is_free_space_costmap(x, y):
            return x, y

        steps = int(search_radius / step)
        for r in range(1, steps + 1):
            for dx in [-r * step, 0, r * step]:
                for dy in [-r * step, 0, r * step]:
                    nx, ny = x + dx, y + dy
                    mx = int((nx - origin_x) / res)
                    my = int((ny - origin_y) / res)
                    if mx < 0 or my < 0 or mx >= width or my >= height:
                        continue
                    occ_value = self.global_costmap_data.data[my * width + mx]
                    if occ_value == 0:
                        return nx, ny

        return x, y

    # ----------------------
    # 초기 위치 설정
    # ----------------------
    def set_initial_pose(self):
        x, y, yaw_deg = 2.4, 13.1, 270.0

        q = self.get_quaternion_from_yaw(yaw_deg)

        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = x
        initial_pose.pose.position.y = y
        initial_pose.pose.orientation.x = q[0]
        initial_pose.pose.orientation.y = q[1]
        initial_pose.pose.orientation.z = q[2]
        initial_pose.pose.orientation.w = q[3]

        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info("📍 초기 위치 설정 완료.")

    def get_quaternion_from_yaw(self, yaw_deg):
        yaw_rad = math.radians(yaw_deg)
        return tf_transformations.quaternion_from_euler(0, 0, yaw_rad)

    # ----------------------
    # 단일 목적지 이동 처리
    # ----------------------
    def des_coor_callback(self, msg: PoseArray):
        if not (self.map_ready and self.pose_ready):
            self.get_logger().warn("🛑 Map/Pose not ready yet, skipping navigation.")
            return

        if not msg.poses:
            self.get_logger().warn("⚠️ No goals received in /des_coor")
            return

        # 첫 번째 목적지만 사용
        goal_pose = msg.poses[0]

        # 목적지 free space 보정
        new_x, new_y = self.find_nearest_free_space(goal_pose.position.x, goal_pose.position.y)
        if (new_x, new_y) != (goal_pose.position.x, goal_pose.position.y):
            self.get_logger().warn(
                f"⚠️ Goal adjusted to nearest free space: "
                f"({goal_pose.position.x:.2f}, {goal_pose.position.y:.2f}) -> ({new_x:.2f}, {new_y:.2f})"
            )

        goal_stamped = PoseStamped()
        goal_stamped.header.frame_id = "map"
        goal_stamped.header.stamp = self.get_clock().now().to_msg()
        goal_stamped.pose = goal_pose
        goal_stamped.pose.position.x = new_x
        goal_stamped.pose.position.y = new_y
        self.get_logger().info(
            f"🚀 Navigating to goal: ({new_x:.2f}, {new_y:.2f}, yaw={self.get_yaw_from_quaternion(goal_pose.orientation):.2f} rad)"
        )
        # 주행 시작
        self.navigator.goToPose(goal_stamped)
        self.task_running = True

        # 모니터링 타이머 설정
        if self.monitor_timer is None:
            self.monitor_timer = self.create_timer(1.0, self.monitor_navigation)

    # ----------------------
    # 여러 경유지 경로 계산
    # ----------------------
    def pre_des_coor_callback(self, msg: PoseArray):
        if not (self.map_ready and self.pose_ready and self.global_costmap_ready):
            self.get_logger().warn("🛑 Map/Pose/Costmap not ready yet, skipping path request.")
            return

        if not msg.poses:
            self.get_logger().warn("⚠️ No goals received in /pre_des_coor")
            return

        # Start pose 보정
        start_x, start_y = self.find_nearest_free_space(
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y
        )

        # goals_stamped = []
        # for idx, p in enumerate(msg.poses):
        #     new_x, new_y = self.find_nearest_free_space(p.position.x, p.position.y)
        #     if (new_x, new_y) != (p.position.x, p.position.y):
        #         self.get_logger().warn(
        #             f"⚠️ Goal {idx} adjusted to nearest free space: "
        #             f"({p.position.x:.2f}, {p.position.y:.2f}) -> ({new_x:.2f}, {new_y:.2f})"
        #         )
        #     ps = PoseStamped()
        #     ps.header.frame_id = "map"
        #     ps.header.stamp = self.get_clock().now().to_msg()
        #     ps.pose = p
        #     ps.pose.position.x = new_x
        #     ps.pose.position.y = new_y
        #     goals_stamped.append(ps)

        # self.get_logger().info(f"목적지 개수: {len(goals_stamped)}")
        # self.get_logger().info("📡 Calling getPathThroughPoses() ...")

        for idx, p in enumerate(msg.poses):
            cost = self.get_costmap_value(p.position.x, p.position.y)
            self.get_logger().info(f"🎯 Goal {idx} at ({p.position.x:.2f}, {p.position.y:.2f}) → cost = {cost}")


        valid_goals = []
        for idx, p in enumerate(msg.poses):
            if not self.is_valid_goal(p.position.x, p.position.y):
                self.get_logger().warn(f"⛔ Goal {idx} ({p.position.x:.2f}, {p.position.y:.2f})는 유효하지 않아 제외됨")
                continue

            # free space 보정 (선택)
            new_x, new_y = self.find_nearest_free_space(p.position.x, p.position.y)

            if not self.is_valid_goal(new_x, new_y):
                self.get_logger().warn(f"⛔ 보정된 Goal {idx} ({new_x:.2f}, {new_y:.2f})도 유효하지 않아 제외됨")
                continue

            
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose = p
            ps.pose.position.x = new_x
            ps.pose.position.y = new_y

            valid_goals.append(ps)

        if not valid_goals:
            self.get_logger().error("❌ 유효한 목적지가 없어 경로 생성을 중단합니다.")
            return

        # path 요청
        start_pose_stamped = PoseStamped()
        start_pose_stamped.header.frame_id = "map"
        start_pose_stamped.header.stamp = self.get_clock().now().to_msg()
        start_pose_stamped.pose = self.current_pose.pose
        start_pose_stamped.pose.position.x = start_x
        start_pose_stamped.pose.position.y = start_y
        start_pose_stamped.pose.position.z = 0.0

        # orientation 명시적 지정
        q = quaternion_from_euler(0, 0, 0.0)  # yaw 0도
        start_pose_stamped.pose.orientation.x = q[0]
        start_pose_stamped.pose.orientation.y = q[1]
        start_pose_stamped.pose.orientation.z = q[2]
        start_pose_stamped.pose.orientation.w = q[3]

        # path = self.navigator.getPathThroughPoses(start_pose_stamped, goals_stamped)
        path = self.navigator.getPathThroughPoses(start_pose_stamped, valid_goals)

        # print(f"Path length: {len(path.poses)}")
        # print(f"Path: {[ (p.pose.position.x, p.pose.position.y) for p in path.poses ]}")

        # 하나씩 path 테스트
        # for i, g in enumerate(valid_goals):
        #     path = self.navigator.getPathThroughPoses(start_pose_stamped, [g])
        #     if not path or not path.poses:
        #         self.get_logger().error(f"🚫 Goal {i} ({g.pose.position.x:.2f}, {g.pose.position.y:.2f}) 는 단독으로도 경로 생성 실패")

        if path is None:
            self.get_logger().error("❌ getPathThroughPoses returned None")
            self.get_logger().info(f"start: ({start_x:.2f},{start_y:.2f})")
            for i, g in enumerate(valid_goals):
                self.get_logger().info(f"goal[{i}]: ({g.pose.position.x}, {g.pose.position.y})")
            return
        

        if not path or not path.poses:
            self.get_logger().warn(f"🚫 Skipping goal at ({p.position.x:.2f}, {p.position.y:.2f}) - invalid or out of bounds")

            self.get_logger().error(f"❌ Failed to get path from Nav2 (start: ({start_x:.2f},{start_y:.2f}), goals: {[ (g.pose.position.x, g.pose.position.y) for g in valid_goals]})")

            return

        step = 10  # 10개에 하나씩만 선택 (튜닝 가능)
        reduced_path = path.poses[::step]

        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.poses = [pose.pose for pose in reduced_path]

        self.path_pub.publish(pose_array)
        self.get_logger().info("유효한 path 발행 완료")

        if self.clear_path_timer is None:
            self.clear_path_timer = self.create_timer(2, self.clear_path)

    # ========== 0.5초 후 빈 path 발행 ==========
    def clear_path(self):
        empty_path = PoseArray()
        empty_path.header.frame_id = "map"
        empty_path.header.stamp = self.get_clock().now().to_msg()
        empty_path.poses = []
        self.path_pub.publish(empty_path)
        self.get_logger().info("🧹 빈 path 발행 (Aggregator 초기화 유도)")
        self.destroy_timer(self.clear_path_timer)
        self.clear_path_timer = None

    def get_costmap_value(self, x, y):
        """
        global_costmap에서 (x, y)의 costmap 값을 반환
        """
        if self.global_costmap_data is None:
            self.get_logger().warn("⚠️ 아직 global costmap 데이터를 수신하지 못했습니다.")
            return None

        res = self.global_costmap_data.info.resolution
        origin_x = self.global_costmap_data.info.origin.position.x
        origin_y = self.global_costmap_data.info.origin.position.y
        width = self.global_costmap_data.info.width
        height = self.global_costmap_data.info.height

        mx = int((x - origin_x) / res)
        my = int((y - origin_y) / res)

        if mx < 0 or my < 0 or mx >= width or my >= height:
            self.get_logger().warn(f"🚫 ({x:.2f}, {y:.2f})는 costmap 범위 밖입니다.")
            return None

        idx = my * width + mx
        cost = self.global_costmap_data.data[idx]
        self.get_logger().info(f"📌 Costmap value at ({x:.2f}, {y:.2f}) → index ({mx},{my}) = {cost}")
        return cost


    # ----------------------
    # 주행 모니터링
    # ----------------------
    def monitor_navigation(self):
        if not self.task_running:
            return

        if not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f"📏 남은 거리: {feedback.distance_remaining:.2f} m")
            return

        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("✅ 목적지 도착 완료 (SUCCEEDED)")
            msg = String()
            msg.data = "ARRIVED"
            self.arrived_pub.publish(msg)
            self.get_logger().info("ARRIVED 메시지 발행 완료")
            
        elif result == TaskResult.CANCELED:
            self.get_logger().warn("⏹ 주행이 취소됨 (CANCELED)")

        elif result == TaskResult.FAILED:
            self.get_logger().error("❌ 주행 실패 (FAILED)")

        else:
            self.get_logger().warn(f"⚠️ 알 수 없는 결과 코드: {result}")

        self.task_running = False
        if self.monitor_timer:
            self.destroy_timer(self.monitor_timer)
            self.monitor_timer = None


    # ----------------------
    # 일시정지 명령 처리
    # ----------------------
    def pause_command_callback(self, msg: String):
        cmd = msg.data.strip().upper()
        if cmd == 'PS' and self.task_running:
            self.navigator.cancelTask()
            self.get_logger().info("⏸ Navigation paused by command.")


    def is_valid_goal(self, x, y, threshold=90):
        """
        (x, y)가 global costmap 상에서 free space(0)인지 확인
        """
        if self.global_costmap_data is None:
            self.get_logger().warn("⚠️ Costmap 데이터 없음")
            return False

        res = self.global_costmap_data.info.resolution
        origin_x = self.global_costmap_data.info.origin.position.x
        origin_y = self.global_costmap_data.info.origin.position.y
        width = self.global_costmap_data.info.width
        height = self.global_costmap_data.info.height

        mx = int((x - origin_x) / res)
        my = int((y - origin_y) / res)

        if mx < 0 or my < 0 or mx >= width or my >= height:
            self.get_logger().warn(f"❌ ({x:.2f}, {y:.2f})는 costmap 영역 밖입니다.")
            return False

        occ_value = self.global_costmap_data.data[my * width + mx]
        # if occ_value != 0:
        #     self.get_logger().warn(f"❌ ({x:.2f}, {y:.2f})는 장애물 또는 unknown 영역입니다. value={occ_value}")
        #     return False

        if occ_value < 0 or occ_value > threshold:
            self.get_logger().warn(f"❌ ({x:.2f}, {y:.2f})는 장애물 또는 unknown 영역입니다. value={occ_value}")
            return False
        self.get_logger().info(f"✅ ({x:.2f}, {y:.2f})는 유효한 목표입니다. value={occ_value}")
        return True

    def get_yaw_from_quaternion(self, q):
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )
        return yaw


def main():
    rclpy.init()
    node = ArcsNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
