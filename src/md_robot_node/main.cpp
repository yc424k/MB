#include "md_robot_node/main.hpp" // 클래스 선언이 포함된 헤더
#include "md_robot_node/com.hpp"
#include "md_robot_node/robot.hpp"
#include <memory>
#include <chrono>

// 다른 파일에서 MdRobotNode의 기능에 접근하기 위한 전역 포인터
std::shared_ptr<MdRobotNode> g_node_ptr = nullptr;

// 외부 파일(com.cpp)에 정의된 변수 참조
extern PID_ROBOT_MONITOR_t curr_pid_robot_monitor;
extern uint32_t pid_response_receive_count;

// 이 노드에서 상태를 관리하고 다른 파일에서 참조할 변수
volatile uint32_t pid_request_cmd_vel_count = 0;

// 프로그램 진입점
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MdRobotNode>();
    g_node_ptr = node; // 전역 포인터에 할당
    rclcpp::spin(node);
    rclcpp::shutdown();
    g_node_ptr = nullptr;
    return 0;
}

// --- MdRobotNode 클래스 멤버 함수 구현부 ---

MdRobotNode::MdRobotNode() : Node("md_robot_node"), byCntComStep(0), fgInitsetting(INIT_SETTING_STATE_NONE)
{
    // 파라미터 선언 및 로드
    declare_parameters();
    load_parameters();
    print_parameters();

    // 통신 초기화
    std::string port = this->get_parameter("serial_port").as_string();
    int baudrate = this->get_parameter("serial_baudrate").as_int();
    if (InitSerialComm(port, baudrate) == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Serial communication initialization failed.");
        rclcpp::shutdown();
        return;
    }

    // Publisher, Subscriber, Timer 생성
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&MdRobotNode::cmdVelCallBack, this, std::placeholders::_1));
    reset_pos_sub_ = this->create_subscription<std_msgs::msg::Bool>("reset_position", 10, std::bind(&MdRobotNode::resetPositionCallBack, this, std::placeholders::_1));
    reset_alarm_sub_ = this->create_subscription<std_msgs::msg::Bool>("reset_alarm", 10, std::bind(&MdRobotNode::resetAlarmCallBack, this, std::placeholders::_1));
    md_robot_message1_pub = this->create_publisher<md::msg::MdRobotMsg1>("md_robot_message1", 10);
    md_robot_message2_pub = this->create_publisher<md::msg::MdRobotMsg2>("md_robot_message2", 10);
    init_delay_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MdRobotNode::init_sequence_start, this));
}

void MdRobotNode::declare_parameters()
{
    this->declare_parameter<int>("use_MDUI", 1);
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("serial_baudrate", 250000);
    this->declare_parameter<int>("reverse_direction", 0);
    this->declare_parameter<int>("maxrpm", 500);
    this->declare_parameter<int>("enable_encoder", 1);
    this->declare_parameter<int>("slow_start", 60);
    this->declare_parameter<int>("slow_down", 60);
    this->declare_parameter<double>("wheel_length", 0.471);
    this->declare_parameter<double>("reduction", 26.85);
    this->declare_parameter<double>("wheel_radius", 0.075);
    this->declare_parameter<int>("encoder_PPR", 1024);
}

void MdRobotNode::load_parameters()
{
    this->get_parameter("use_MDUI", robotParamData.use_MDUI);
    this->get_parameter("serial_baudrate", robotParamData.nBaudrate);
    this->get_parameter("reverse_direction", robotParamData.reverse_direction);
    this->get_parameter("maxrpm", robotParamData.nMaxRPM);
    this->get_parameter("enable_encoder", robotParamData.enable_encoder);
    this->get_parameter("slow_start", robotParamData.nSlowstart);
    this->get_parameter("slow_down", robotParamData.nSlowdown);
    this->get_parameter("wheel_length", robotParamData.nWheelLength);
    this->get_parameter("reduction", robotParamData.nGearRatio);
    this->get_parameter("wheel_radius", robotParamData.wheel_radius);
    this->get_parameter("encoder_PPR", robotParamData.encoder_PPR);

    robotParamData.nIDPC = 1;   // PC ID
    robotParamData.nIDMDUI = 2; // MDUI ID
    robotParamData.nIDMDT = 3;  // MDT ID
    robotParamData.nDiameter = (int)(robotParamData.wheel_radius * 2.0 * 1000.0);
    robotParamData.nRMID = robotParamData.use_MDUI ? robotParamData.nIDMDUI : robotParamData.nIDMDT;
}

void MdRobotNode::print_parameters()
{
    RCLCPP_INFO(this->get_logger(), "----------------------------------");
    RCLCPP_INFO(this->get_logger(), " Version 2.6 (ROS2 Port)");
    RCLCPP_INFO(this->get_logger(), " 2024.07.11");
    RCLCPP_INFO(this->get_logger(), "----------------------------------");
    if (robotParamData.use_MDUI == 1)
    {
        RCLCPP_INFO(this->get_logger(), "----------------------------------");
        RCLCPP_INFO(this->get_logger(), " Using MDUI(RS232)");
        RCLCPP_INFO(this->get_logger(), "----------------------------------");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "----------------------------------");
        RCLCPP_INFO(this->get_logger(), " Direct MDT(RS485)");
        RCLCPP_INFO(this->get_logger(), "----------------------------------");
    }
    RCLCPP_INFO(this->get_logger(), "Serial port             : %s", this->get_parameter("serial_port").as_string().c_str());
    RCLCPP_INFO(this->get_logger(), "Baudrate                : %d bps", robotParamData.nBaudrate);
    RCLCPP_INFO(this->get_logger(), "Diameter(mm)            : %d", robotParamData.nDiameter);
    RCLCPP_INFO(this->get_logger(), "Wheel Radius(m)         : %f", robotParamData.wheel_radius);
    RCLCPP_INFO(this->get_logger(), "WheelLength(m)          : %f", robotParamData.nWheelLength);
    RCLCPP_INFO(this->get_logger(), "Reduction rate          : %d", robotParamData.nGearRatio);
    RCLCPP_INFO(this->get_logger(), "Max RPM                 : %d", robotParamData.nMaxRPM);
    if (robotParamData.reverse_direction == 0)
    {
        RCLCPP_INFO(this->get_logger(), "Robot direction         : Forward");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Robot direction         : Reverse");
    }
    if (robotParamData.enable_encoder == 0)
    {
        RCLCPP_INFO(this->get_logger(), "motor position detection: hall sensor");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "motor position detection: encoder");
        if (robotParamData.use_MDUI == 1)
        {
            RCLCPP_INFO(this->get_logger(), " PPR                    : %d", robotParamData.encoder_PPR);
        }
    }
    RCLCPP_INFO(this->get_logger(), "Slow start              : %d", robotParamData.nSlowstart);
    RCLCPP_INFO(this->get_logger(), "Slow down               : %d", robotParamData.nSlowdown);
}

void MdRobotNode::cmdVelCallBack(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (fgInitsetting == INIT_SETTING_STATE_OK)
    {
        velCmdRcvCount++;
        velCmdUpdateCount++;
        goal_cmd_speed = msg->linear.x;
        goal_cmd_ang_speed = msg->angular.z;
    }
}

void MdRobotNode::resetPositionCallBack(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data)
    {
        RCLCPP_INFO(this->get_logger(), "Reset Position");
        reset_pos_flag = true;
    }
}

void MdRobotNode::resetAlarmCallBack(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data)
    {
        RCLCPP_INFO(this->get_logger(), "Reset Alarm");
        reset_alarm_flag = true;
    }
}

void MdRobotNode::VelCmdRcvTimeoutCallback()
{
    static uint32_t old_velCmdRcvCount = 0;

    if (velCmdRcvCount == old_velCmdRcvCount)
    {
        goal_cmd_speed = 0;
        goal_cmd_ang_speed = 0;
        if (remote_pc_connection_state)
        {
            velCmdUpdateCount++;
            remote_pc_connection_state = false;
        }
    }
    else
    {
        old_velCmdRcvCount = velCmdRcvCount;
        if (!remote_pc_connection_state)
        {
            remote_pc_connection_state = true;
        }
    }

    if (pid_request_cmd_vel_count > 5)
    {
        if (mdui_mdt_connection_state)
            mdui_mdt_connection_state = false;
    }
    else if (pid_request_cmd_vel_count == 2)
    {
        if (!mdui_mdt_connection_state)
            mdui_mdt_connection_state = true;
    }
}

void MdRobotNode::init_sequence_start()
{
    init_delay_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Starting motor parameter initialization...");
    byCntInitStep = SETTING_PARAM_STEP_PID_PNT_VEL_CMD;
    fgInitsetting = INIT_SETTING_STATE_NONE;
    init_loop_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MdRobotNode::init_loop, this));
}

void MdRobotNode::init_loop()
{
    ReceiveSerialData();
    InitMotorParameter();
    if (fgInitsetting != INIT_SETTING_STATE_NONE)
    {
        init_loop_timer_->cancel();
        if (fgInitsetting == INIT_SETTING_STATE_OK)
        {
            RCLCPP_INFO(this->get_logger(), "[Init done]");
            start_main_loops();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Robot initialization failed!");
            rclcpp::shutdown();
        }
    }
}

void MdRobotNode::start_main_loops()
{
    mdui_mdt_connection_state = true;
    remote_pc_connection_state = false;
    pid_request_cmd_vel_count = 0;
    byCntComStep = 0;
    main_loop_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MdRobotNode::main_loop, this));
    connection_check_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MdRobotNode::VelCmdRcvTimeoutCallback, this));
}

void MdRobotNode::main_loop()
{
    static bool old_mdui_mdt_connection_state = this->mdui_mdt_connection_state;
    static bool old_remote_pc_connection_state = this->remote_pc_connection_state;

    if (!remote_pc_connection_state && old_remote_pc_connection_state)
    {
        RCLCPP_INFO(this->get_logger(), "Remote PC connection state: error!!!");
        pid_request_cmd_vel_count = 0;
    }
    else if (remote_pc_connection_state && !old_remote_pc_connection_state)
    {
        RCLCPP_INFO(this->get_logger(), "Remote PC connection state: Ok");
        pid_request_cmd_vel_count = 0;
    }
    old_remote_pc_connection_state = remote_pc_connection_state;

    if (!mdui_mdt_connection_state && old_mdui_mdt_connection_state)
    {
        RCLCPP_INFO(this->get_logger(), "%s connection state: error!", robotParamData.use_MDUI ? "MDUI" : "MDT");
    }
    else if (mdui_mdt_connection_state && !old_mdui_mdt_connection_state)
    {
        RCLCPP_INFO(this->get_logger(), "%s connection state: Ok", robotParamData.use_MDUI ? "MDUI" : "MDT");
    }
    old_mdui_mdt_connection_state = mdui_mdt_connection_state;

    ReceiveSerialData();
    RequestRobotStatus();
}

void MdRobotNode::InitMotorParameter()
{
    switch (byCntInitStep)
    {
    case SETTING_PARAM_STEP_PID_PNT_VEL_CMD:
    {
        PID_PNT_VEL_CMD_t cmd_data, *p;
        RCLCPP_INFO(this->get_logger(), "[SET] PID_PNT_VEL_CMD(PID NO: %d)", PID_PNT_VEL_CMD);
        p = &cmd_data;
        p->enable_id1 = 1;
        p->rpm_id1 = 0;
        p->enable_id2 = 1;
        p->rpm_id2 = 0;
        if (robotParamData.use_MDUI == 1)
        {
            p->req_monitor_id = REQUEST_PID_ROBOT_MONITOR;
        }
        else
        {
            p->req_monitor_id = REQUEST_PNT_MAIN_DATA;
        }
        pid_response_receive_count = 0;
        pid_request_cmd_vel_count = 1;
        PutMdData(PID_PNT_VEL_CMD, robotParamData.nRMID, (const uint8_t *)&cmd_data, sizeof(cmd_data));
        byCntInitStep = SETTING_PARAM_WAIT;
        break;
    }
    case SETTING_PARAM_WAIT:
    {
        if (pid_response_receive_count > 0)
        {
            pid_response_receive_count = 0;
            byCntInitStep = SETTING_PARAM_STEP_PID_ROBOT_PARAM;
        }
        else
        {
            check_connection_retry_count++;
            if (check_connection_retry_count >= MAX_CONNECTION_CHECK_COUNT)
            {
                RCLCPP_INFO(this->get_logger(), "!!! Error RS232(MDUI) or RS485(MDT) !!!");
                fgInitsetting = INIT_SETTING_STATE_ERROR;
                byCntInitStep = SETTING_PARAM_STEP_DONE;
            }
            else
            {
                byCntInitStep = SETTING_PARAM_STEP_PID_PNT_VEL_CMD;
            }
        }
        break;
    }
    case SETTING_PARAM_STEP_PID_ROBOT_PARAM:
    {
        if (robotParamData.use_MDUI == 1)
        {
            PID_ROBOT_PARAM_t cmd_data, *p;
            RCLCPP_INFO(this->get_logger(), "[SET] SETTING_PARAM_STEP_PID_ROBOT_PARAM(PID NO: %d)", PID_ROBOT_PARAM);
            p = &cmd_data;
            p->nDiameter = (uint16_t)robotParamData.nDiameter;
            p->nWheelLength = (uint16_t)(robotParamData.nWheelLength * 1000.0);
            p->nGearRatio = (uint16_t)robotParamData.nGearRatio;
            RCLCPP_INFO(this->get_logger(), "D: %d, L: %d, R: %d", p->nDiameter, p->nWheelLength, p->nGearRatio);
            PutMdData(PID_ROBOT_PARAM, MID_MDUI, (const uint8_t *)p, sizeof(cmd_data));
        }
        byCntInitStep = SETTING_PARAM_STEP_PID_POSI_RESET;
        break;
    }
    case SETTING_PARAM_STEP_PID_POSI_RESET:
    {
        uint8_t dummy;
        RCLCPP_INFO(this->get_logger(), "[SET] PID_POSI_RESET(PID NO: %d)", PID_POSI_RESET);
        dummy = 0;
        PutMdData(PID_POSI_RESET, robotParamData.nRMID, (const uint8_t *)&dummy, sizeof(dummy));
        byCntInitStep = SETTING_PARAM_STEP_PID_SLOW_START;
        break;
    }
    case SETTING_PARAM_STEP_PID_SLOW_START:
    {
        PID_SLOW_START_t cmd_data, *p;
        RCLCPP_INFO(this->get_logger(), "[SET] PID_SLOW_START(PID NO: %d)", PID_SLOW_START);
        p = &cmd_data;
        p->value = robotParamData.nSlowstart;
        PutMdData(PID_SLOW_START, robotParamData.nRMID, (const uint8_t *)p, sizeof(cmd_data));
        byCntInitStep = SETTING_PARAM_STEP_PID_SLOW_DOWN;
        break;
    }
    case SETTING_PARAM_STEP_PID_SLOW_DOWN:
    {
        PID_SLOW_DOWN_t cmd_data, *p;
        RCLCPP_INFO(this->get_logger(), "[SET] PID_SLOW_DOWN(PID NO: %d)", PID_SLOW_DOWN);
        p = &cmd_data;
        p->value = robotParamData.nSlowdown;
        PutMdData(PID_SLOW_DOWN, robotParamData.nRMID, (const uint8_t *)p, sizeof(cmd_data));
        byCntInitStep = SETTING_PARAM_STEP_PID_INV_SIGH_CMD;
        break;
    }
    case SETTING_PARAM_STEP_PID_INV_SIGH_CMD:
    {
        uint8_t cmd_data = (robotParamData.reverse_direction == 0) ? 1 : 0;
        RCLCPP_INFO(this->get_logger(), "[SET] PID_INV_SIGN_CMD(PID NO: %d)", PID_INV_SIGN_CMD);
        PutMdData(PID_INV_SIGN_CMD, robotParamData.nRMID, &cmd_data, 1);
        byCntInitStep = SETTING_PARAM_STEP_PID_INV_SIGH_CMD2;
        break;
    }
    case SETTING_PARAM_STEP_PID_INV_SIGH_CMD2:
    {
        uint8_t cmd_data = (robotParamData.reverse_direction == 0) ? 0 : 1;
        RCLCPP_INFO(this->get_logger(), "[SET] PID_INV_SIGN_CMD2(PID NO: %d)", PID_INV_SIGN_CMD2);
        PutMdData(PID_INV_SIGN_CMD2, robotParamData.nRMID, &cmd_data, 1);
        byCntInitStep = SETTING_PARAM_STEP_PID_USE_EPOSI;
        break;
    }
    case SETTING_PARAM_STEP_PID_USE_EPOSI:
    {
        uint8_t cmd_data = (robotParamData.enable_encoder == 0) ? 0 : 1;
        RCLCPP_INFO(this->get_logger(), "[SET] PID_USE_POSI(PID NO: %d)", PID_USE_POSI);
        PutMdData(PID_USE_POSI, robotParamData.nRMID, &cmd_data, 1);
        byCntInitStep = SETTING_PARAM_STEP_PID_PPR;
        break;
    }
    case SETTING_PARAM_STEP_PID_PPR:
    {
        if (robotParamData.use_MDUI == 1 && robotParamData.enable_encoder == 1)
        {
            PID_PPR_t cmd_data, *p;
            RCLCPP_INFO(this->get_logger(), "[SET] PID_PPR(PID NO: %d)", PID_PPR);
            p = &cmd_data;
            p->PPR = robotParamData.encoder_PPR;
            PutMdData(PID_PPR, robotParamData.nRMID, (const uint8_t *)&cmd_data, sizeof(PID_PPR_t));
        }
        byCntInitStep = SETTING_PARAM_STEP_DONE;
        if (pid_request_cmd_vel_count == 2)
        {
            fgInitsetting = INIT_SETTING_STATE_OK;
        }
        else
        {
            fgInitsetting = INIT_SETTING_STATE_ERROR;
            if (robotParamData.use_MDUI == 1)
            {
                RCLCPP_INFO(this->get_logger(), "!!! Error RS232 interface !!!");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "!!! Error RS485 interface !!!");
            }
        }
        break;
    }
    default:
        break;
    }
}

void MdRobotNode::RequestRobotStatus()
{
    int16_t *pGoalRPMSpeed;

    switch (byCntComStep)
    {
    case 0:
    {
        if (velCmdUpdateCount > 0)
        {
            velCmdUpdateCount = 0;
            PID_PNT_VEL_CMD_t pid_pnt_vel_cmd, *p;
            if (mdui_mdt_connection_state == true)
            {
                pGoalRPMSpeed = RobotSpeedToRPMSpeed(goal_cmd_speed, goal_cmd_ang_speed, this->robotParamData);
            }
            else
            {
                int16_t zero_speeds[2] = {0, 0};
                pGoalRPMSpeed = zero_speeds;
            }
            RCLCPP_INFO(this->get_logger(), "Goal %.2f, %.2f, RPM L:%d, R:%d", goal_cmd_speed, goal_cmd_ang_speed, pGoalRPMSpeed[0], pGoalRPMSpeed[1]);
            p = &pid_pnt_vel_cmd;
            p->enable_id1 = 1;
            p->rpm_id1 = pGoalRPMSpeed[0];
            p->enable_id2 = 1;
            p->rpm_id2 = pGoalRPMSpeed[1];
            if (robotParamData.use_MDUI == 1)
            {
                p->req_monitor_id = REQUEST_PID_ROBOT_MONITOR;
            }
            else
            {
                p->req_monitor_id = REQUEST_PNT_MAIN_DATA;
            }
            PutMdData(PID_PNT_VEL_CMD, robotParamData.nRMID, (const uint8_t *)&pid_pnt_vel_cmd, sizeof(pid_pnt_vel_cmd));
            pid_request_cmd_vel_count++;
        }

        if (robotParamData.use_MDUI == 1)
        {
            if (curr_pid_robot_monitor.byPlatStatus.bits.bEmerSW == 1)
            {
                byCntComStep = 3;
                break;
            }
        }
        if (reset_pos_flag == true || reset_alarm_flag == true)
        {
            byCntComStep = 3;
        }
        else
        {
            byCntComStep = 4;
        }
        break;
    }
    case 3:
    {
        if (robotParamData.use_MDUI == 1)
        {
            if (curr_pid_robot_monitor.byPlatStatus.bits.bEmerSW == 1)
            {
                PID_PNT_TQ_OFF_t pid_pnt_tq_off;
                pid_pnt_tq_off.enable_id1 = 1;
                pid_pnt_tq_off.enable_id2 = 1;
                pid_pnt_tq_off.req_monitor_id = REQUEST_PNT_MAIN_DATA;
                PutMdData(PID_PNT_TQ_OFF, robotParamData.nRMID, (const uint8_t *)&pid_pnt_tq_off, sizeof(pid_pnt_tq_off));
            }
        }
        if (reset_pos_flag == true)
        {
            uint8_t dummy = 0;
            reset_pos_flag = false;
            PutMdData(PID_POSI_RESET, robotParamData.nRMID, &dummy, sizeof(dummy));
        }
        else if (reset_alarm_flag == true)
        {
            uint8_t cmd_pid = CMD_ALARM_RESET;
            reset_alarm_flag = false;
            PutMdData(PID_COMMAND, robotParamData.nRMID, &cmd_pid, 1);
        }
        byCntComStep = 0;
        break;
    }
    case 4:
    {
        uint8_t request_pid;
        if (robotParamData.use_MDUI == 0)
        {
            request_pid = PID_IO_MONITOR;
            RCLCPP_INFO(this->get_logger(), "REQ: PID_IO_MONITOR");
        }
        else
        {
            request_pid = PID_ROBOT_MONITOR2;
            RCLCPP_INFO(this->get_logger(), "REQ: PID_ROBOT_MONITOR2");
        }
        PutMdData(PID_REQ_PID_DATA, robotParamData.nRMID, &request_pid, 1);
        byCntComStep = 0;
        break;
    }
    default:
        byCntComStep = 0;
        break;
    }
}