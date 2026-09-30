# 문제 해결

## `map` 또는 `odom` frame이 존재하지 않음

증상:

```text
Invalid frame ID "map"
Invalid frame ID "odom"
Timed out waiting for transform from base_link to map
```

점검:

```bash
ros2 topic hz /Odometry
ros2 topic hz /odom
ros2 topic echo /map_to_odom --once
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

FAST-LIO localization, transform fusion, wheel odometry 중 어느 단계가 끊겼는지 순서대로 확인합니다. 서로 다른 워크스페이스의 `install/setup.bash`를 중첩 source하지 마십시오.

## RViz robot과 Gazebo robot 위치가 다름

주요 원인:

- 잘못된 `/initialpose`
- PCD prior map과 2D occupancy map의 좌표 불일치
- `map -> odom` 또는 `odom -> base_link` 중복 publisher
- ICP가 충분히 겹치지 않는 scan으로 잘못 수렴
- SLAM recovery 순간 live/frozen transform 차이

점검:

```bash
ros2 topic echo /fastlio/icp_fitness
ros2 topic echo /fastlio/localization_ok
ros2 topic echo /plot/recovery_align_error
ros2 run tf2_tools view_frames
```

초기 pose는 Gazebo spawn 위치에 가깝게 지정하고, `/cur_scan_in_map`이 prior map 벽과 겹치는지 확인합니다.

## `Robot is out of bounds of the costmap`

로봇 pose가 occupancy map 영역 밖에 있거나 map origin/initial pose가 틀린 경우입니다. Nav2 goal을 반복해서 주기 전에 localization을 먼저 고쳐야 합니다.

확인할 파일:

- `src/nav2/2dmap/fastlio_map_2d_2.yaml`
- `pcd/map_lite2.pcd`
- `src/base_model/launch/scout_gazebo.launch.py`

## goal을 주어도 path가 생성되지 않음

- 시작 footprint 또는 goal이 lethal/inflated cell에 있는지 확인합니다.
- global costmap이 robot pose를 포함하는지 확인합니다.
- `map -> base_link` TF age를 확인합니다.
- planner server가 active인지 확인합니다.

```bash
ros2 lifecycle get /planner_server
ros2 topic echo /global_costmap/costmap --once
ros2 action list | grep navigate_to_pose
```

## 토글이 계속 `nav2`에 머묾

다음 토픽을 동시에 확인합니다.

```bash
ros2 topic echo /plot/auto_armed
ros2 topic echo /hybrid/goal_cached
ros2 topic echo /fastlio/icp_fitness
ros2 topic echo /plot/fitness_bad
ros2 topic echo /slam_degradation_flag
```

`auto_armed=false`이면 goal cache, fitness 수신 또는 5초 arming delay 조건이 충족되지 않은 것입니다. 현재 설정에서 낮은 `C_degrad`만으로는 반드시 토글되지 않으며 ICP fitness gate가 주 조건입니다.

## NeuPAN에서 Nav2로 복구되지 않음

```bash
ros2 topic echo /fastlio/icp_fitness
ros2 topic echo /plot/recovery_align_error
ros2 topic echo /plot/recovery_align_good
```

fitness가 `0.39`를 넘더라도 frozen/live alignment error가 `0.30 m`보다 크면 복구가 보류됩니다. 이 보호 조건을 무리하게 완화하면 goal 근처 pose jump와 급회전이 다시 발생할 수 있습니다.

## goal 성공 로그 후 로봇이 계속 움직임

`cmd_vel` publisher 중복과 stale command를 확인합니다.

```bash
ros2 topic info /cmd_vel -v
ros2 topic echo /cmd_vel_mux/source
ros2 topic echo /cmd_vel
```

정상 hybrid 구성에서는 최종 `/cmd_vel`의 주 publisher가 `cmd_vel_mux`여야 합니다. 도착 시 `override`가 짧게 선택되고 zero twist가 유지되는지 확인합니다.

## `Transform data too old`

모든 노드의 `use_sim_time`이 같고 `/clock`이 발행되는지 확인합니다.

```bash
ros2 topic hz /clock
ros2 param get /controller_server use_sim_time
ros2 param get /fast_lio_mapping use_sim_time
```

이전 simulation 프로세스가 남아 있거나 서로 다른 `ROS_DOMAIN_ID`의 노드를 섞어 관찰하지 않도록 주의합니다.

## 장애물 충돌 또는 전복

- local costmap의 `/scan`이 실제 장애물을 포함하는지 확인합니다.
- footprint와 inflation radius가 Scout 크기와 맞는지 확인합니다.
- 목표를 장애물 내부나 너무 가까운 곳에 두지 않습니다.
- 최고 선속도/각속도를 올릴 때 controller acceleration과 costmap update rate도 함께 검토합니다.
- 첫 시험은 낮은 속도와 넓은 공간에서 수행합니다.

