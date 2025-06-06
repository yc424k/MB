#include "md_robot_node/robot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "md/msg/MdRobotMsg1.hpp"
#include "md/msg/MdRobotMsg2.hpp"
#include "md_robot_node/global.hpp"
#include <algorithm>

#define VELOCITY_CONSTANT_VALUE 9.5492743
#define LEFT 0
#define RIGHT 1
#define constrain(amt, low, high) ((amt) <= (low) ? (low) : ((amt) >= (high) ? (high) : (amt)))

int16_t *RobotSpeedToRPMSpeed(double linear, double angular, const ROBOT_PARAMETER_t &robotParam)
{
    double wheel_radius = robotParam.wheel_radius;
    double wheel_separation = robotParam.nWheelLength;
    double reduction = (double)robotParam.nGearRatio;
    double wheel_velocity_cmd[2];
    static int16_t goal_rpm_spped[2];

    wheel_velocity_cmd[LEFT] = linear - (angular * wheel_separation / 2.0);
    wheel_velocity_cmd[RIGHT] = linear + (angular * wheel_separation / 2.0);

    wheel_velocity_cmd[LEFT] = constrain(wheel_velocity_cmd[LEFT] * VELOCITY_CONSTANT_VALUE / wheel_radius * reduction, -robotParam.nMaxRPM, robotParam.nMaxRPM);
    wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT] * VELOCITY_CONSTANT_VALUE / wheel_radius * reduction, -robotParam.nMaxRPM, robotParam.nMaxRPM);

    goal_rpm_spped[0] = (int16_t)(wheel_velocity_cmd[LEFT]);
    goal_rpm_spped[1] = (int16_t)(wheel_velocity_cmd[RIGHT]);

    return goal_rpm_spped;
}

md::msg::MdRobotMsg1 MakeMDRobotMessage1(const PID_PNT_MAIN_DATA_t *pData, const PID_IO_MONITOR_t &io_monitor, rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger)
{
    static rclcpp::Time previous_time(0, 0, clock->get_clock_type());
    rclcpp::Time current_time = clock->now();
    double interval_time = (current_time - previous_time).seconds();
    previous_time = current_time;

    md::msg::MdRobotMsg1 msg;
    msg.interval_time = interval_time;
    msg.motor1_pos = pData->mtr_pos_id1;
    msg.motor2_pos = pData->mtr_pos_id2;
    msg.motor1_rpm = pData->rpm_id1;
    msg.motor2_rpm = pData->rpm_id2;
    msg.motor1_state = pData->mtr_state_id1.val;
    msg.motor2_state = pData->mtr_state_id2.val;
    msg.input_voltage = static_cast<float>(io_monitor.input_voltage / 10.0);

    RCLCPP_INFO(logger, "interval time1: %f, pos1: %d, pos2: %d, rpm1: %d rpm2: %d, input voltage: %f",
                msg.interval_time, msg.motor1_pos, msg.motor2_pos, msg.motor1_rpm, msg.motor2_rpm, msg.input_voltage);

    return msg;
}

md::msg::MdRobotMsg2 MakeMDRobotMessage2(const PID_ROBOT_MONITOR_t *pData, const PID_ROBOT_MONITOR2_t &monitor2, const ROBOT_PARAMETER_t &robotParam, rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger)
{
    static rclcpp::Time previous_time(0, 0, clock->get_clock_type());
    rclcpp::Time current_time = clock->now();
    double interval_time = (current_time - previous_time).seconds();
    previous_time = current_time;

    md::msg::MdRobotMsg2 msg;
    msg.interval_time = interval_time;
    msg.x_pos = pData->lTempPosi_x;
    msg.y_pos = pData->lTempPosi_y;
    msg.angule = pData->sTempTheta;

    if (robotParam.reverse_direction == 0)
    {
        msg.us_1 = pData->byUS1;
        msg.us_2 = pData->byUS2;
    }
    else
    {
        msg.us_1 = pData->byUS2;
        msg.us_2 = pData->byUS1;
    }

    msg.platform_state = pData->byPlatStatus.val;
    msg.linear_velocity = pData->linear_velocity;
    msg.angular_velocity = pData->angular_velocity;
    msg.input_voltage = static_cast<float>(monitor2.sVoltIn / 10.0);

    RCLCPP_INFO(logger, "interval time2: %f, input_voltage: %f", msg.interval_time, msg.input_voltage);

    return msg;
}