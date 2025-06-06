# CMake 최소 버전과 프로젝트 이름을 설정합니다. ROS2는 3.8 이상을 권장합니다.
cmake_minimum_required(VERSION 3.8)
project(md)

# C++ 표준을 17로 설정합니다. ROS2 Humble은 C++17을 요구합니다.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ament_cmake와 필요한 ROS2 패키지들을 찾습니다.
# ROS2에서는 각 패키지를 개별적으로 찾아야 합니다.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)


# 'serial' 패키지는 ros-humble-serial-driver 또는 유사 패키지를 설치해야 합니다.
# sudo apt-get install ros-humble-serial-driver
find_package(serial_driver REQUIRED)

# 메시지/서비스 생성을 위해 필요한 패키지를 찾습니다.
find_package(rosidl_default_generators REQUIRED)

# 사용자 정의 메시지 파일(.msg)들을 정의합니다.
set(msg_files
  "msg/MdRobotMsg1.msg"   
  "msg/MdRobotMsg2.msg"   
  "msg/RobotPose.msg"     
)

# ROS2 인터페이스(메시지, 서비스)를 생성합니다.
# 이 함수가 catkin의 add_message_files, generate_messages 역할을 대체합니다.
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

# 빌드 시 필요한 의존성(라이브러리 포함)을 링크합니다.
# ROS2에서는 ament_target_dependencies를 사용하여 의존성을 명확하게 지정합니다.
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

# 설치(install) 규칙을 정의합니다.
# 실행 파일(노드) 설치
install(TARGETS md_robot_node
  DESTINATION lib/${PROJECT_NAME}
)

# 헤더 파일 설치
install(DIRECTORY include/
  DESTINATION include
)

# launch 파일 설치
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# package.xml 파일 설치
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# 이 패키지의 리소스(공유 파일)를 등록하고 의존성을 export합니다.
# 이 함수가 catkin_package()의 역할을 대체하며, 항상 CMakeLists.txt의 맨 마지막에 위치해야 합니다.
ament_package()