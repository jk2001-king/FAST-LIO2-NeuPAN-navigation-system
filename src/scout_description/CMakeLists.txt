cmake_minimum_required(VERSION 3.8)
project(scout_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 빌드 시스템 호출
find_package(ament_cmake REQUIRED)

# 원본 파일이 요구한 launch, meshes, urdf 폴더를 모두 설치합니다
install(DIRECTORY launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()