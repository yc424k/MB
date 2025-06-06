#pragma once

#include "md_robot_node/com.hpp" // ROBOT_PARAMETER_t와 같은 타입을 알기 위해 com.hpp를 먼저 포함합니다.
#include <cstdint>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "md/msg/md_robot_msg1.hpp"
#include "md/msg/md_robot_msg2.hpp"
#include "md_robot_node/global.hpp" // global.hpp가 필요하다면 여기에 포함

class MdRobotNode : public rclcpp::Node
{
public:
    // 생성자
    MdRobotNode();

    // 다른 파일에서 접근해야 하는 멤버들은 public으로 설정
    ROBOT_PARAMETER_t robotParamData;
    rclcpp::Publisher<md::msg::MdRobotMsg1>::SharedPtr md_robot_message1_pub;
    rclcpp::Publisher<md::msg::MdRobotMsg2>::SharedPtr md_robot_message2_pub;

private:
    // 콜백 함수들
    void cmdVelCallBack(const geometry_msgs::msg::Twist::SharedPtr msg);
    void resetPositionCallBack(const std_msgs::msg::Bool::SharedPtr msg);
    void resetAlarmCallBack(const std_msgs::msg::Bool::SharedPtr msg);
    void VelCmdRcvTimeoutCallback();

    // 내부 로직 및 상태 관리 함수
    void init_sequence_start();
    void init_loop();
    void start_main_loops();
    void main_loop();
    void declare_parameters();
    void load_parameters();
    void print_parameters();
    void InitMotorParameter();
    void RequestRobotStatus();

    // 내부에서만 사용하는 멤버 변수들
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_pos_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_alarm_sub_;
    rclcpp::TimerBase::SharedPtr init_delay_timer_;
    rclcpp::TimerBase::SharedPtr init_loop_timer_;
    rclcpp::TimerBase::SharedPtr main_loop_timer_;
    rclcpp::TimerBase::SharedPtr connection_check_timer_;

    SETTINNG_PARAM_STEP_t byCntInitStep;
    uint16_t byCntComStep;
    uint32_t velCmdUpdateCount = 0;
    uint32_t velCmdRcvCount = 0;
    volatile bool mdui_mdt_connection_state = false;
    volatile bool remote_pc_connection_state = false;
    INIT_SETTING_STATE_t fgInitsetting;
    uint16_t check_connection_retry_count = 0;
    double goal_cmd_speed = 0.0;
    double goal_cmd_ang_speed = 0.0;
    bool reset_pos_flag = false;
    bool reset_alarm_flag = false;
};