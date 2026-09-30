# 시스템 아키텍처

## 설계 목표

FAST-LIO2와 Nav2만 사용하는 시스템은 LiDAR의 기하학적 특징이 부족한 구간에서 scan-to-map registration이 불안정해지면 `map` 기준 로봇 자세가 튈 수 있습니다. 이 프로젝트는 정상 상태의 전역 주행 성능은 유지하면서, 정합 열화 구간에서는 발산 중인 map localization을 제어 기준에서 일시적으로 분리하는 것을 목표로 합니다.

핵심 원칙은 다음 세 가지입니다.

1. 정합 상태를 단일 Boolean이 아닌 연속 지표와 보조 gate로 관측합니다.
2. 전환 순간의 좌표 변환을 고정하여 기존 global path를 wheel-odometry 기준으로 이어갑니다.
3. 복구 시 live localization과 fallback trajectory의 정렬을 확인한 뒤 Nav2로 돌아갑니다.

## TF와 좌표계

정상 상태의 주요 체인은 다음과 같습니다.

```text
map ── global localization / transform_fusion ── odom ── robot odometry ── base_link
```

- `/Odometry`: FAST-LIO2가 발행하는 LiDAR-IMU odometry
- `/odom`: Gazebo/Scout wheel odometry
- `/localization`: `map -> base_link`로 융합된 localization 결과
- `/neupan/current_odom`: `odom_switcher`가 연속성을 보존해 내보내는 NeuPAN용 odometry

fallback 시 새로운 wheel frame을 만들지 않습니다. 전환 직전 출력 pose와 `/odom`의 차이를 SE(2) offset으로 저장하고, wheel odometry에 이 offset을 적용해 계속 `odom` frame의 연속적인 pose를 발행합니다.

## 구성 노드

### `auto_toggle`

입력:

| 토픽 | 의미 |
|---|---|
| `/Odometry` | FAST-LIO2 pose와 미분 속도 |
| `/odom` | wheel odometry 속도 |
| `/slam_res_mean` | ICP fitness에서 변환된 residual proxy |
| `/fastlio/icp_fitness` | global scan-to-map ICP fitness |
| `/fastlio/localization_ok` | global localization 성공 상태 |
| `/hybrid/goal_cached` | 유효한 navigation goal 존재 여부 |
| `/plot/recovery_align_error` | frozen/live map alignment 차이 |

출력:

- `/slam_degradation_flag`
- `/plot/c_degrad`, `/plot/c_degrad_raw`
- `/plot/c_res`, `/plot/c_vel`, `/plot/c_z`
- `/plot/auto_armed`, `/plot/fitness_bad`, `/plot/c_bad`, `/plot/loc_bad`

통합 지표는 아래와 같습니다.

```text
c_res = w_res * S_res / S_norm
c_vel = w_vel * |v_slam - v_wheel| / V_norm
c_z   = w_z   * |z_slam - z_baseline| / Z_norm
C_raw = c_res + c_vel + c_z + localization penalty
C_degrad[k] = alpha * C_raw[k] + (1 - alpha) * C_degrad[k-1]
```

Z 항은 절대 높이가 아니라 정상 주행 중 천천히 갱신되는 baseline과의 drift입니다. 초기 transient로 인한 오검출을 줄이기 위해 goal이 cache되고 fitness가 수신된 뒤 5초가 지나야 auto-toggle이 armed 됩니다.

### `toggle_manager`

- RViz의 `/goal_pose`와 Nav2의 `/plan`을 저장합니다.
- 열화 진입 직전 정상 `map <-> odom` transform을 frozen transform으로 보존합니다.
- map-frame global path를 frozen transform으로 `odom`에 재투영해 `/neupan_plan_input_odom`으로 발행합니다.
- NeuPAN goal을 `/neupan/final_goal`로 전달합니다.
- fallback 중 goal 근접 상태를 확인하고 `/cmd_vel_override`로 확실히 정지시킵니다.
- 복구 시 alignment 조건이 만족되면 기존 Nav2 goal을 action으로 다시 전송합니다.

### `odom_switcher`

- 정상: `/Odometry`를 연속적인 `/neupan/current_odom`으로 변환
- 열화: `/odom` wheel odometry를 offset 정렬하여 같은 출력으로 변환
- 복구: 현재 출력과 복구된 FAST-LIO2 pose 사이 offset을 다시 계산

### `cmd_vel_mux`

```text
normal   : /cmd_vel_nav2   -> /cmd_vel
degraded : /cmd_vel_neupan -> /cmd_vel
override : /cmd_vel_override -> /cmd_vel
```

선택된 source가 0.25초 이상 갱신되지 않으면 오래된 non-zero command를 재사용하지 않고 zero twist를 발행합니다. 현재 source는 `/cmd_vel_mux/source`로 확인할 수 있습니다.

## 상태 전이

```text
START
  |
  | goal cached + fitness received + arming delay
  v
NAV2 / HEALTHY
  |
  | fitness < 0.35 and localization unstable for 1.0 s
  | or FAST-LIO odometry timeout
  v
NEUPAN / DEGRADED
  |
  | fitness > 0.39 for 1.5 s
  | and recovery alignment error <= 0.30 m
  v
NAV2 RECOVERY
  |
  | resend cached NavigateToPose goal
  v
NAV2 / HEALTHY
```

### 슈도코드

```text
loop at 10 Hz:
    update C_degrad from residual, velocity disagreement, and z drift
    publish all diagnostic components

    if goal and fitness are not ready for arming:
        keep NAV2 mode
        continue

    if FAST-LIO odometry is stale:
        switch_to_degraded()

    if mode == NAV2:
        if low_fitness and unstable_localization persist for degrade_confirm:
            freeze healthy map/odom transform
            align wheel odometry to current output pose
            reproject cached map path into odom
            switch cmd_vel source to NeuPAN

    else if mode == NEUPAN:
        if recovered_fitness and alignment_good persist for recover_confirm:
            align FAST-LIO pose to current output pose
            switch cmd_vel source to Nav2
            resend cached Nav2 goal

    if near_goal or navigation completed:
        publish zero-velocity override
```

## 기본 임계값

| 파라미터 | 값 | 역할 |
|---|---:|---|
| `degrade_threshold` | 1.0 | 분석용 `C_degrad` 열화 기준 |
| `recover_threshold` | 0.7 | 분석용 `C_degrad` 복구 기준 |
| `fitness_degrade_threshold` | 0.35 | 온라인 열화 gate |
| `fitness_recover_threshold` | 0.39 | 온라인 복구 gate |
| `degrade_confirm_sec` | 1.0 s | 열화 지속 확인 |
| `recover_confirm_sec` | 1.5 s | 복구 지속 확인 |
| `arming_delay_sec` | 5.0 s | 초기 transient 무시 |
| `recovery_align_threshold` | 0.30 m | 복구 정렬 허용 오차 |
| `arrive_distance_threshold` | 0.9 m | fallback 도착 근접 기준 |

launch에서 전달한 값이 노드 내부 기본값보다 우선합니다. 최종 실험값은 `master_IICC.launch.py`를 기준으로 합니다.

