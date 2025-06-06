#pragma once

#include "md_robot_node/com.hpp" // ROBOT_PARAMETER_t와 같은 타입을 알기 위해 com.hpp를 먼저 포함합니다.
#include "rclcpp/rclcpp.hpp"
#include "md/msg/md_robot_msg1.hpp"
#include "md/msg/md_robot_msg2.hpp"
#include "md/msg/robot_pose.hpp"

// robot.cpp에 정의된 함수들의 원형(선언)
int16_t* RobotSpeedToRPMSpeed(double linear, double angular, const ROBOT_PARAMETER_t& robotParam);
md::msg::MdRobotMsg1 MakeMDRobotMessage1(const PID_PNT_MAIN_DATA_t *pData, const PID_IO_MONITOR_t& io_monitor, rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger);
md::msg::MdRobotMsg2 MakeMDRobotMessage2(const PID_ROBOT_MONITOR_t *pData, const PID_ROBOT_MONITOR2_t& monitor2, const ROBOT_PARAMETER_t& robotParam, rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger);