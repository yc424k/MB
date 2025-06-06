# CMake 최소 버전과 프로젝트 이름을 설정합니다.
cmake_minimum_required(VERSION 3.8)
project(md)

# C++ 표준을 17로 설정합니다.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ament_cmake와 필요한 ROS2 패키지들을 찾습니다.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(serial_driver REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 사용자 정의 메시지 파일(.msg)들을 정의합니다.
set(msg_files
  "msg/MdRobotMsg1.msg"
  "msg/MdRobotMsg2.msg"
  "msg/RobotPose.msg"
)

# ROS2 인터페이스(메시지)를 생성합니다.
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
)

# 헤더 파일이 있는 'include' 디렉터리를 포함시킵니다.
include_directories(include)

# 실행 파일(노드)을 정의합니다.
add_executable(md_robot_node
  src/md_robot_node/main.cpp
  src/md_robot_node/com.cpp
  src/md_robot_node/robot.cpp
)

# ament_target_dependencies는 외부 패키지와의 의존성을 설정합니다.
ament_target_dependencies(md_robot_node
  rclcpp
  rclpy
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  tf2_ros
  serial_driver
)

# =================================================================
#               !!! 가장 중요한 수정 사항 !!!
# =================================================================
# 이 target_link_libraries 명령은 실행 파일(md_robot_node)이
# 이 패키지 내부에서 생성된 메시지 라이브러리(${PROJECT_NAME}__cpp)와
# 링크되도록 명시적으로 지정합니다.
# 이 부분이 빌드 순서 문제를 해결하는 핵심입니다.
target_link_libraries(md_robot_node
  PRIVATE
  ${PROJECT_NAME}__cpp
)
# =================================================================

# 설치(install) 규칙을 정의합니다.
install(TARGETS md_robot_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# package.xml 파일 설치
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# ament_package()는 항상 CMakeLists.txt의 맨 마지막에 위치해야 합니다.
ament_package()