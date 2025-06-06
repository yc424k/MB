#pragma once

#include <cstdint>
#include "rclcpp/rclcpp.hpp"
#include "md/msg/MdRobotMsg1.hpp"
#include "md/msg/MdRobotMsg2.hpp"
#include "md/msg/RobotPose.hpp"
#include "md_robot_node/global.hpp"

// robot.cpp에 정의된 함수들의 원형(선언)
int16_t* RobotSpeedToRPMSpeed(double linear, double angular, const ROBOT_PARAMETER_t& robotParam);
md::msg::MdRobotMsg1 MakeMDRobotMessage1(const PID_PNT_MAIN_DATA_t *pData, const PID_IO_MONITOR_t& io_monitor, rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger);
md::msg::MdRobotMsg2 MakeMDRobotMessage2(const PID_ROBOT_MONITOR_t *pData, const PID_ROBOT_MONITOR2_t& monitor2, const ROBOT_PARAMETER_t& robotParam, rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger);