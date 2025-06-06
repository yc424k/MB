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
# 이 함수는 ${PROJECT_NAME}__rosidl_typesupport_cpp 라는 라이브러리 타겟을 생성합니다.
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

# 실행 파일을 메시지로부터 생성된 실제 라이브러리 타겟에 직접 링크합니다.
# 이 방법이 빌드 순서와 헤더 경로 문제를 모두 해결하는 최종 해결책입니다.
target_link_libraries(md_robot_node
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

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
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# ament_package()는 항상 CMakeLists.txt의 맨 마지막에 위치해야 합니다.
ament_package()