# 환경 설정과 실행

## 요구 환경

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 및 `gazebo_ros_pkgs`
- Python 3.10
- PCL, Eigen, OpenMP
- CUDA/PyTorch 환경은 NeuPAN 모델 실행 환경에 맞춰 준비

ROS 작업 전 conda가 활성화돼 있다면 비활성화하는 것을 권장한다. conda Python이 `/opt/ros/humble`의 Python 패키지를 가리면 `rclpy` 또는 ament 관련 import 오류가 발생할 수 있다.

```bash
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
```

## ROS 의존성

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-gazebo-ros-pkgs ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-topic-tools \
  ros-humble-pointcloud-to-laserscan ros-humble-tf-transformations \
  ros-humble-pcl-ros ros-humble-pcl-conversions
```

Python 의존성은 사용하는 NeuPAN checkout과 GPU 환경에 맞춰 설치한다. localization 스크립트는 최소한 NumPy, SciPy, Open3D 및 transforms3d를 사용한다.

```bash
/usr/bin/python3 -m pip install --user numpy scipy open3d transforms3d
```

NeuPAN 모델 의존성은 `src/neupan_ros2`의 원본 README도 함께 확인한다.

## Clone과 빌드

이 저장소의 일부 launch/config는 저장소 루트 기준 상대경로를 사용하므로, 실행 전에 반드시 저장소 루트로 이동한다.

```bash
git clone https://github.com/jk2001-king/FAST-LIO2-NeuPAN-navigation-system.git ~/IICC_ws
cd ~/IICC_ws

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

빌드 후 다음 패키지가 보여야 한다.

```bash
colcon list
ros2 pkg prefix slam_toggle
ros2 pkg prefix fast_lio_localization
ros2 pkg prefix neupan_ros2
```

## 전체 실행

```bash
cd ~/IICC_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=26
ros2 launch master_IICC.launch.py
```

다른 world를 사용할 때:

```bash
ros2 launch master_IICC.launch.py \
  enable_hybrid:=true \
  world_path:=src/base_model/worlds/feature_corridor.world
```

baseline 실험:

```bash
ros2 launch master_IICC.launch.py enable_hybrid:=false
```

## 시작 순서

통합 launch는 startup race를 줄이기 위해 다음 순서로 노드를 시작한다.

| 시각 | 구성 |
|---:|---|
| 즉시 | Gazebo, Scout model, pointcloud fixer |
| 3 s | map server |
| 7 s | FAST-LIO localization, RViz |
| 10 s | toggle manager, odom switcher, auto toggle, cmd mux |
| 13 s | Nav2 |
| 16 s | QoS bridge, NeuPAN |

Gazebo가 느린 장비에서는 첫 실행 시 TF와 sensor topic이 안정화된 뒤 goal을 지정한다.

## 초기 위치 설정

1. RViz fixed frame이 `map`인지 확인한다.
2. global map과 현재 scan의 대략적인 위치가 겹치는지 확인한다.
3. **2D Pose Estimate**를 사용해 Gazebo의 실제 spawn 위치와 같은 위치를 지정한다.
4. `/fastlio/icp_fitness`가 안정적으로 발행되는지 확인한다.
5. 그 다음 Nav2 Goal을 지정한다.

초기 pose는 scan과 prior map이 겹치지 않는 위치에 주면 ICP가 낮은 fitness로 실패하며 robot이 costmap 밖에 있는 것으로 보일 수 있다.

## 정상 동작 점검

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /Odometry
ros2 topic hz /odom
ros2 topic hz /scan
ros2 topic echo /fastlio/icp_fitness --once
ros2 topic echo /cmd_vel_mux/source
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

## 지도 교체

3D PCD와 2D occupancy map은 같은 mapping 결과와 좌표계를 사용해야 한다.

- PCD: `pcd/map_lite2.pcd`
- YAML/PGM: `src/nav2/2dmap/fastlio_map_2d_2.yaml`, `fastlio_map_2d_2.pgm`

다른 지도를 사용할 때는 `master_IICC.launch.py`, FAST-LIO localization config, Nav2 map 경로를 함께 변경하고 RViz에서 scan overlap을 다시 확인한다.

