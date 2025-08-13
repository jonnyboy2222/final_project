import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point,PoseArray, Pose, Quaternion
from std_msgs.msg import String
import socket
import struct
import serial
import threading
import json
import tf_transformations
import math
#상점1 : x5.57,y12.88
#상점2 : x4.3,y8.6
#상점3 : x4.27,y6.40
#문전 : x-0.2,y6.0
#문후 : x-0.2,y0.45
#상점4 : x1.8,y0.0
#상점5 : x3.2,y0.0
#상점6 : x:5.5y0.0
#------------------
# charging station : x:3.0 y:13.0
# gate2: x:3.0 y:10.0
# 식당1 : x:3.0 y:7.0
# 편의점 : x:5.5 y:13.0
# info : x:4.5, y:8.5
# cafe1 : x: 4.5 y: 6.0
# gate1 : x: 6.0 y 11.5
# 화장실1 : x: 6.0 y: 8.5
# gate3 : x: 6.0 y:5.0
# 화장실2 : x: 0.0 y: 0.0
# 면세점1: x: 1.0 y: 0.0
# 면세점2: x: 2.5 y: 0.0
# 면세점3: x:4.0 y: 0.0
# 식당2 : x: 5.5 y: 0.0
class ArcsCommander(Node):
    def __init__(self):
        super().__init__('arcs_commander')

        self.state = "WAITING"

        # 상태 전환 테이블
        self.TRANSITIONS = {
            "WAITING": {
                "FW": "FOLLOWING",
                "LD": "LEADING",
                "RT": "RETURNING",
                "ED": "LEADING",
            },
            "FOLLOWING": {
                "PS": "PAUSING",
                "RT": "RETURNING",
            },
            "LEADING": {
                "PS": "PAUSING",
                "RT": "RETURNING",
            },
            "RETURNING": {
                "PS": "PAUSING",
                "FW": "FOLLOWING",
                "ED": "LEADING",
                "LD": "LEADING",
            },
            "PAUSING": {
                "FW": "FOLLOWING",
                "LD": "LEADING",
                "RT": "RETURNING",
                "ED": "LEADING",
            }
        }

        # ROS Publishers
        self.des_coor_pub = self.create_publisher(PoseArray, '/des_coor', 10)
        self.pre_des_coor_pub = self.create_publisher(PoseArray, '/pre_des_coor', 10)
        self.vision_data_pub = self.create_publisher(Point, '/vision_data', 10)
        self.mode_pub = self.create_publisher(String, '/arcs_mode', 10)
        self.pause_command_pub = self.create_publisher(String, '/pause_command', 10)

        # UDP 설정
        self.udp_ip = '0.0.0.0'
        self.udp_port = 23458
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind((self.udp_ip, self.udp_port))
        self.get_logger().info(f'📡 UDP Listening on {self.udp_ip}:{self.udp_port}')


    # 상태 변경 함수
    def set_state(self, new_state):
        if self.state != new_state:
            self.state = new_state
            msg = String()
            msg.data = self.state
            self.mode_pub.publish(msg)
            self.get_logger().info(f"🔄 상태 변경 → {self.state}")

    # UDP 리스너
    def udp_listener(self):
        while True:
            try:
                data, _ = self.udp_sock.recvfrom(1024)
                self.handle_packet(data)
            except Exception as e:
                self.get_logger().error(f'UDP receive error: {e}')

    # 패킷 처리
    def handle_packet(self, data: bytes):
        if len(data) < 7:
            return

        header = data[0]
        length = int.from_bytes(data[1:5], 'big')
        command = data[5:7].decode('utf-8')
        payload = data[7:7+length]
        self.get_logger().info(f"📥 Command: {command}, Length: {length}, Before: {self.state}")
        
        # 명령별 동작
        if command in ("ED", "LD", "RT"):
            self.publish_des_coor(payload)

        elif command == "CK":
            self.publish_pre_des_coor(payload)
            print("pre_des_coor published")

        elif command == "VD" and self.state == "FOLLOWING":
            self.publish_vision_data(payload)
            
        elif command == "PS" and self.state in ("FOLLOWING", "LEADING", "RETURNING"):
            msg = String()
            msg.data = 'PS'
            self.pause_command_pub.publish(msg)
            self.get_logger().info("🛑 Pause command published")

        # 상태 전환 처리
        if command in self.TRANSITIONS.get(self.state, {}):
            new_state = self.TRANSITIONS[self.state][command]
            self.set_state(new_state)
        self.get_logger().info(f"📥 Command: {command}, Length: {length}, After: {self.state}")
    # 목적지 좌표 퍼블리시
    def publish_des_coor(self, payload):
        try:
            # UDP에서 받은 payload → JSON 파싱
            goals_list = json.loads(payload.decode('utf-8'))  # {"0": [x, y, theta], ...}
            print("des_coord_goals_list:", goals_list)
            pose_array = PoseArray()
            pose_array.header.stamp = self.get_clock().now().to_msg()
            pose_array.header.frame_id = "map"

            # key 순서대로 Pose 생성
            for des_coor in goals_list:
                x = float(des_coor[0])
                y = float(des_coor[1])
                theta_deg = float(des_coor[2])
                theta_rad = math.radians(theta_deg)
                q = tf_transformations.quaternion_from_euler(0, 0, theta_rad)

                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                pose_array.poses.append(pose)

            # 기존 단일 Pose2D 대신 PoseArray 발행
            self.des_coor_pub.publish(pose_array)
            self.get_logger().info(f"📤 Published /goal_pose_list with {len(pose_array.poses)} goals")

        except Exception as e:
            self.get_logger().error(f"❌ Failed to parse goal list: {e}")

    def publish_pre_des_coor(self, payload):
        try:
            
            goals_list = json.loads(payload.decode('utf-8'))
            print("pre_des_coord_goals_list:", goals_list)
            pose_array = PoseArray()
            pose_array.header.stamp = self.get_clock().now().to_msg()
            pose_array.header.frame_id = "map"

            for des_coor in goals_list:
                x = float(des_coor[0])
                y = float(des_coor[1])
                theta_deg = float(des_coor[2])
                theta_rad = math.radians(theta_deg)
                q = tf_transformations.quaternion_from_euler(0, 0, theta_rad)
                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                pose_array.poses.append(pose)

            # CK 전용 토픽 발행
            self.pre_des_coor_pub.publish(pose_array)
            self.get_logger().info(f"📤 Published /des_coor_ck with {len(pose_array.poses)} goals")

        except Exception as e:
            self.get_logger().error(f"❌ Failed to parse CK goal list: {e}")

    # Vision 데이터 퍼블리시
    def publish_vision_data(self, payload: bytes):
        try:
            # JSON 파싱
            data = json.loads(payload.decode('utf-8'))
            # self.get_logger().info(f"📥 Received vision_data: {data}")

            # 유효성 검사
            if not isinstance(data, dict):
                self.get_logger().warn("⚠️ vision_data must be a JSON object")
                return
            if "center_point" not in data or "distance" not in data:
                self.get_logger().warn("⚠️ vision_data JSON missing keys: center_point, distance")
                return

            cp = data["center_point"]
            dist = data["distance"]
            # self.get_logger().info(f"📥 center_point: {cp}, distance: {dist}")
            # if (not isinstance(cp, (list, tuple)) or len(cp) != 2 or
            #     not all(isinstance(v, (int, float)) for v in cp) or
            #     not isinstance(dist, (int, float))):
            #     self.get_logger().warn("⚠️ vision_data JSON schema invalid")
            #     return

            # Point 타입 메시지 생성
            pt = Point()

                    # 빈 리스트인 경우 → (320, 0, 1) 전송
            if cp == []:
                self.get_logger().warn("⚠️ center_point is empty → sending (320, 0, 1)")
                pt.x, pt.y, pt.z = 160.0, 0.0, 1.0
            else:
                # if (not isinstance(cp, (list, tuple)) or len(cp) != 2 or
                #     not all(isinstance(v, (int, float)) for v in cp) or
                #     not isinstance(dist, (int, float))):
                #     self.get_logger().warn("⚠️ vision_data JSON schema invalid")
                #     return
                pt.x = float(cp[0])
                pt.y = float(cp[1])
                pt.z = float(dist)

            self.vision_data_pub.publish(pt)
            self.get_logger().info(
                f"📤 Published /vision_data: center=({pt.x:.2f}, {pt.y:.2f}), distance={pt.z:.2f}"
            )

        except Exception as e:
            self.get_logger().error(f"❌ Failed to handle vision data (JSON only): {e}")

def main():
    rclpy.init()
    node = ArcsCommander()
    threading.Thread(target=node.udp_listener, daemon=True).start()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
