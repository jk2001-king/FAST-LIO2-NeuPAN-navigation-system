cmake_minimum_required(VERSION 3.8)
project(base_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 빌드 필수 패키지
find_package(ament_cmake REQUIRED)

# ==========================================
# 설치 규칙 (Install Rules)
# ==========================================
# urdf 폴더를 설치하여 런치 파일에서 접근할 수 있도록 함
install(DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

# ROS 2 패키지 등록
ament_package()