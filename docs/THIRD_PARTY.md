# 외부 프로젝트 및 라이선스

이 저장소는 아래 오픈소스 프로젝트를 기반으로 수정한 코드를 포함한다. 각 디렉터리의 원본 README와 LICENSE가 우선하며, 해당 코드의 재배포와 사용은 원 프로젝트 라이선스를 따라야 한다.

| Component | Upstream | Local path |
|---|---|---|
| FAST_LIO_ROS2 | https://github.com/Ericsii/FAST_LIO_ROS2 | `src/FAST_LIO_ROS2` |
| FAST_LIO_LOCALIZATION_ROS2 | https://github.com/myeongw002/FAST_LIO_LOCALIZATION_ROS2 | `src/FAST_LIO_LOCALIZATION_ROS2` |
| NeuPAN ROS 2 | https://github.com/KevinLADLee/neupan_ros2 | `src/neupan_ros2` |
| Livox ROS Driver 2 | https://github.com/Livox-SDK/livox_ros_driver2 | `src/livox_ros_driver2` |
| Livox Gazebo simulation | https://github.com/LCAS/livox_laser_simulation_ros2 | `src/ros2_livox_simulation` |
| pcd2pgm | https://github.com/LihanChen2004/pcd2pgm | `src/pcd2pgm` |

프로젝트 통합 과정에서 launch/config, FAST-LIO localization metric 발행, NeuPAN topic/frame 연동, Gazebo sensor/model 설정이 수정되었다.

`src/slam_toggle`, 최상위 통합 launch, 분석 스크립트 및 프로젝트 문서에 적용할 별도 라이선스는 저장소 소유자가 명시적으로 결정해야 한다. 현재는 임의의 오픈소스 라이선스를 선언하지 않았다.

