import yaml
import cv2

# === 기본 설정 ===
map_yaml_path = "/home/lee/final_project/simulation/academy.yaml"
pgm_path = "/home/lee/final_project/simulation/academy.pgm"
output_world_path = "./simulation/academy.world"

# === 맵 메타데이터 불러오기 ===
with open(map_yaml_path, 'r') as f:
    metadata = yaml.safe_load(f)
resolution = metadata['resolution']
origin = metadata['origin']
origin_x, origin_y = origin[0], origin[1]

# === 맵 이미지 불러오기 및 처리 ===
image = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
height, width = image.shape
_, binary = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY_INV)
contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# === SDF 모델 생성 ===
model_elements = ""
for i, cnt in enumerate(contours):
    x, y, w, h = cv2.boundingRect(cnt)
    center_x = (x + w / 2) * resolution + origin_x
    center_y = (height - (y + h / 2)) * resolution + origin_y
    model_elements += f"""
  <model name="obstacle_{i}">
    <static>true</static>
    <pose>{center_x:.3f} {center_y:.3f} 0.1 0 0 0</pose>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box>
            <size>{w * resolution:.3f} {h * resolution:.3f} 0.2</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>{w * resolution:.3f} {h * resolution:.3f} 0.2</size>
          </box>
        </geometry>
        <material>
          <diffuse>0.5 0.5 0.5 1</diffuse>
        </material>
      </visual>
    </link>
  </model>"""
    
# === 전체 world 파일 포맷으로 래핑 ===
world_content = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="auto_generated_world">
    <physics name="default_physics" type="ignition">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.5 0.5 -1</direction>
    </light>
{model_elements}
  </world>
</sdf>
"""

# === 파일로 저장 ===
with open(output_world_path, 'w') as f:
    f.write(world_content)

print(f"[✔] Gazebo world 저장 완료: {output_world_path}")
