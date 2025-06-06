#pragma once

// 1. ROS2 핵심 헤더
#include "rclcpp/rclcpp.hpp"

// 2. ROS2 메시지 헤더 (경로 및 확장자 변경)
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"

// 3. 표준 C++ 라이브러리 헤더
#include <cstdio>  // stdio.h
#include <cstdlib> // stdlib.h
#include <sstream>
#include <iostream>
#include <cfloat> // float.h
#include <cmath>  // math.h
#include <string>
#include <vector>

// 4. 시리얼 통신 라이브러리 헤더 (새로운 라이브러리로 변경)
#include "serial_driver/serial_driver.hpp"

// 5. 'using namespace std;' 제거
// 헤더 파일에서 'using namespace'를 사용하면
// 이 파일을 include하는 모든 파일에 영향을 주어 이름 충돌을 일으킬 수 있으므로
// 사용하지 않는 것이 표준적인 C++ 관례입니다.