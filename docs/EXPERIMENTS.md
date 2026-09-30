# 실험과 데이터 분석

## 실험 목적

LiDAR 특징이 풍부한 시작/종료 구간과 특징이 부족한 넓은 복도를 연결하여 registration degradation을 유도했다. LiDAR perception range를 제한해 복도 중앙에서 유효한 기하 특징이 감소하도록 구성했다.

실험 world:

```text
src/base_model/worlds/end_feature_wide_mid.world
```

본 실험에는 Gazebo wheel slip을 별도 파라미터로 주입하지 않았다. 따라서 실험 조건은 `wheel-slip environment`가 아니라 **feature-poor wide corridor under reduced perception range**로 기술하는 것이 정확하다.

## 비교군

| 구분 | 실행 인자 | 설명 |
|---|---|---|
| Baseline | `enable_hybrid:=false` | FAST-LIO2 localization + Nav2 only |
| Proposed | `enable_hybrid:=true` | FAST-LIO2/Nav2 + auto-toggle + wheel-odom/NeuPAN fallback |

공정한 비교를 위해 world, 초기 pose, goal, sensor range, Nav2 파라미터를 동일하게 유지한다.

## rosbag 기록

권장 토픽:

```bash
ros2 bag record -o rosbag/com_01 \
  /tf /tf_static /clock \
  /goal_pose /plan /localization /Odometry /odom \
  /cmd_vel /cmd_vel_nav2 /cmd_vel_neupan /cmd_vel_mux/source \
  /fastlio/icp_fitness /fastlio/localization_ok /slam_res_mean \
  /slam_degradation_flag \
  /plot/c_degrad /plot/c_degrad_raw \
  /plot/c_res /plot/c_vel /plot/c_z \
  /plot/auto_armed /plot/recovery_align_error
```

rosbag 디렉터리 전체가 하나의 bag이다. `metadata.yaml` 또는 내부 `.db3` 파일 하나가 아니라 `base_01`, `com_01` 같은 디렉터리 경로를 `ros2 bag info/play`에 전달한다.

## 지표 정의

### Success rate

10회 중 goal 부근에 도달하고 실제 로봇 속도가 0으로 유지되며 navigation action이 성공한 trial의 비율이다. `override` source만 나타난 경우를 자동으로 성공 처리하지 않고, goal distance와 정지 상태를 함께 확인한다.

### Final position error

최종 planar position과 goal 사이의 Euclidean distance이다.

```text
e_final = sqrt((x_end - x_goal)^2 + (y_end - y_goal)^2)
```

여러 trial의 대표값은 RMSE 또는 mean 중 하나를 정해 일관되게 사용해야 한다. 현재 표는 저장된 분석 CSV의 집계값을 사용한다.

### Maximum position jump

map-frame localization의 연속 sample 사이 최대 XY displacement이다.

```text
jump_max = max sqrt((x_k - x_(k-1))^2 + (y_k - y_(k-1))^2)
```

지상 주행 로봇의 navigation 성능을 평가하므로 기본 지표에서는 Z를 제외한다. Z drift는 별도의 `/plot/c_z`와 3D localization 진단으로 분석한다.

### Collision count

각 trial에서 Gazebo 모델이 벽/장애물과 충돌하거나 전복된 사건을 수기로 검증한 횟수이다. 향후 contact sensor topic을 기록하면 자동화할 수 있다.

## 분석 도구

Trajectory 비교:

```bash
python3 tools/plot_trajectory_from_bags.py \
  rosbag/base_03 rosbag/com_12
```

전체 bag metric 추출:

```bash
python3 tools/extract_bag_metrics.py \
  --baseline-glob 'rosbag/base_*' \
  --combined-glob 'rosbag/com_*'
```

스크립트의 CLI는 변경될 수 있으므로 `--help`를 먼저 확인한다.

## 최종 결과

| Metric | Baseline | Proposed | Improvement |
|---|---:|---:|---:|
| Success rate | 50.0% | 90.0% | +80.0% |
| Final position error | 1.032 m | 0.589 m | 42.9% |
| Maximum position jump | 5.399 m | 2.389 m | 55.8% |
| Collision count | 5 | 2 | 60.0% |

![Trajectory comparison](images/trajectory_base03_vs_com11_mapframe.png)

![Degradation state plot](images/C_degrad_graph.png)

## PlotJuggler 권장 필드

- Graph A: `/slam_res_mean`, `/plot/c_vel`, `/plot/c_z`, `/fastlio/icp_fitness`
- Graph B: `/plot/c_degrad`, degrade/recover threshold
- Graph C: `/slam_degradation_flag`, `/cmd_vel_mux/source`, `/plot/auto_armed`

문자열인 `/cmd_vel_mux/source`는 PlotJuggler에서 직접 숫자 step plot으로 쓰기 어려울 수 있으므로, 논문 그래프에서는 `/slam_degradation_flag`를 `Nav2=1`, `NeuPAN=0`으로 변환하거나 후처리 스크립트에서 categorical mode를 숫자로 매핑한다.

