#include "md_robot_node/main.hpp"
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
int main(int argc, char** argv)
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

// 생성자 초기화 리스트를 클래스 선언과 일치시킵니다.
MdRobotNode::MdRobotNode() : Node("md_robot_node"), byCntInitStep(SETTING_PARAM_STEP_PID_PNT_VEL_CMD), byCntComStep(0), fgInitsetting(INIT_SETTING_STATE_NONE)
{
    // 파라미터 선언 및 로드
    this->declare_parameters();
    this->load_parameters();
    this->print_parameters();
    
    // 통신 초기화
    std::string port = this->get_parameter("serial_port").as_string();
    int baudrate = this->get_parameter("serial_baudrate").as_int();
    if(InitSerialComm(port, baudrate) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Serial communication initialization failed.");
        rclcpp::shutdown();
        return;
    }

    // Publisher, Subscriber, Timer 생성
    this->cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&MdRobotNode::cmdVelCallBack, this, std::placeholders::_1));
    this->reset_pos_sub_ = this->create_subscription<std_msgs::msg::Bool>("reset_position", 10, std::bind(&MdRobotNode::resetPositionCallBack, this, std::placeholders::_1));
    this->reset_alarm_sub_ = this->create_subscription<std_msgs::msg::Bool>("reset_alarm", 10, std::bind(&MdRobotNode::resetAlarmCallBack, this, std::placeholders::_1));
    this->md_robot_message1_pub = this->create_publisher<md::msg::MdRobotMsg1>("md_robot_message1", 10);
    this->md_robot_message2_pub = this->create_publisher<md::msg::MdRobotMsg2>("md_robot_message2", 10);
    this->init_delay_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MdRobotNode::init_sequence_start, this));
}

void MdRobotNode::declare_parameters() {
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

void MdRobotNode::load_parameters() {
    this->get_parameter("use_MDUI", this->robotParamData.use_MDUI);
    this->get_parameter("serial_baudrate", this->robotParamData.nBaudrate);
    this->get_parameter("reverse_direction", this->robotParamData.reverse_direction);
    this->get_parameter("maxrpm", this->robotParamData.nMaxRPM);
    this->get_parameter("enable_encoder", this->robotParamData.enable_encoder);
    this->get_parameter("slow_start", this->robotParamData.nSlowstart);
    this->get_parameter("slow_down", this->robotParamData.nSlowdown);
    this->get_parameter("wheel_length", this->robotParamData.nWheelLength);
    this->get_parameter("reduction", this->robotParamData.nGearRatio);
    this->get_parameter("wheel_radius", this->robotParamData.wheel_radius);
    this->get_parameter("encoder_PPR", this->robotParamData.encoder_PPR);

    this->robotParamData.nIDPC = 1;
    this->robotParamData.nIDMDUI = 184;
    this->robotParamData.nIDMDT = 183;
    this->robotParamData.nDiameter = (int)(this->robotParamData.wheel_radius * 2.0 * 1000.0);
    this->robotParamData.nRMID = this->robotParamData.use_MDUI ? this->robotParamData.nIDMDUI : this->robotParamData.nIDMDT;
}

void MdRobotNode::print_parameters() {
    RCLCPP_INFO(this->get_logger(), "----------------------------------");
    RCLCPP_INFO(this->get_logger(), "serial_port: %s", this->get_parameter("serial_port").as_string().c_str());
    RCLCPP_INFO(this->get_logger(), "serial_baudrate: %d", this->robotParamData.nBaudrate);
    RCLCPP_INFO(this->get_logger(), "Diameter(mm)            : %d", this->robotParamData.nDiameter);
    RCLCPP_INFO(this->get_logger(), "Wheel Radius(m)         : %f", this->robotParamData.wheel_radius);
    RCLCPP_INFO(this->get_logger(), "WheelLength(m)          : %f", this->robotParamData.nWheelLength);
    RCLCPP_INFO(this->get_logger(), "Reduction rate          : %d", this->robotParamData.nGearRatio);
    RCLCPP_INFO(this->get_logger(), "Max RPM                 : %d", this->robotParamData.nMaxRPM);
    if (this->robotParamData.reverse_direction == 0) {
        RCLCPP_INFO(this->get_logger(), "Robot direction         : Forward");
    } else {
        RCLCPP_INFO(this->get_logger(), "Robot direction         : Reverse");
    }
    if (this->robotParamData.enable_encoder == 0) {
        RCLCPP_INFO(this->get_logger(), "motor position detection: hall sensor");
    } else {
        RCLCPP_INFO(this->get_logger(), "motor position detection: encoder");
        if (this->robotParamData.use_MDUI == 1) {
            RCLCPP_INFO(this->get_logger(), " PPR                    : %d", this->robotParamData.encoder_PPR);
        }
    }
    RCLCPP_INFO(this->get_logger(), "Slow start              : %d", this->robotParamData.nSlowstart);
    RCLCPP_INFO(this->get_logger(), "Slow down               : %d", this->robotParamData.nSlowdown);
}

void MdRobotNode::cmdVelCallBack(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if(this->fgInitsetting == INIT_SETTING_STATE_OK) {
        this->velCmdRcvCount++;
        this->velCmdUpdateCount++;
        this->goal_cmd_speed = msg->linear.x;
        this->goal_cmd_ang_speed = msg->angular.z;
    }
}

void MdRobotNode::resetPositionCallBack(const std_msgs::msg::Bool::SharedPtr msg) {
    if(msg->data) {
        RCLCPP_INFO(this->get_logger(), "Reset Position");
        this->reset_pos_flag = true;
    }
}

void MdRobotNode::resetAlarmCallBack(const std_msgs::msg::Bool::SharedPtr msg) {
    if(msg->data) {
        RCLCPP_INFO(this->get_logger(), "Reset Alarm");
        this->reset_alarm_flag = true;
    }
}

void MdRobotNode::VelCmdRcvTimeoutCallback() {
    static uint32_t old_velCmdRcvCount = 0;
    if(this->velCmdRcvCount == old_velCmdRcvCount) {
        this->goal_cmd_speed = 0;
        this->goal_cmd_ang_speed = 0;
        if(this->remote_pc_connection_state) {
            this->velCmdUpdateCount++;
            this->remote_pc_connection_state = false;
        }
    } else {
        old_velCmdRcvCount = this->velCmdRcvCount;
        if(!this->remote_pc_connection_state) this->remote_pc_connection_state = true;
    }
    if(pid_request_cmd_vel_count > 5) {
        if(this->mdui_mdt_connection_state) this->mdui_mdt_connection_state = false;
    } else if(pid_request_cmd_vel_count == 2) {
        if(!this->mdui_mdt_connection_state) this->mdui_mdt_connection_state = true;
    }
}

void MdRobotNode::init_sequence_start() {
    this->init_delay_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Starting motor parameter initialization...");
    this->byCntInitStep = SETTING_PARAM_STEP_PID_PNT_VEL_CMD;
    this->fgInitsetting = INIT_SETTING_STATE_NONE;
    this->init_loop_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MdRobotNode::init_loop, this));
}

void MdRobotNode::init_loop() {
    ReceiveSerialData();
    this->InitMotorParameter();
    if(this->fgInitsetting != INIT_SETTING_STATE_NONE) {
        this->init_loop_timer_->cancel();
        if(this->fgInitsetting == INIT_SETTING_STATE_OK) {
            RCLCPP_INFO(this->get_logger(), "[Init done]");
            this->start_main_loops();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Robot initialization failed!");
            rclcpp::shutdown();
        }
    }
}

void MdRobotNode::start_main_loops() {
    this->mdui_mdt_connection_state = true;
    this->remote_pc_connection_state = false;
    pid_request_cmd_vel_count = 0;
    this->byCntComStep = 0;
    this->main_loop_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MdRobotNode::main_loop, this));
    this->connection_check_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MdRobotNode::VelCmdRcvTimeoutCallback, this));
}
    
void MdRobotNode::main_loop() {
    static bool old_mdui_mdt_connection_state = this->mdui_mdt_connection_state;
    static bool old_remote_pc_connection_state = this->remote_pc_connection_state;
    if(!this->remote_pc_connection_state && old_remote_pc_connection_state) {
        RCLCPP_INFO(this->get_logger(), "Remote PC connection state: error!!!");
        pid_request_cmd_vel_count = 0;
    } else if(this->remote_pc_connection_state && !old_remote_pc_connection_state) {
        RCLCPP_INFO(this->get_logger(), "Remote PC connection state: Ok");
        pid_request_cmd_vel_count = 0;
    }
    old_remote_pc_connection_state = this->remote_pc_connection_state;
    if(!this->mdui_mdt_connection_state && old_mdui_mdt_connection_state) {
        RCLCPP_INFO(this->get_logger(), "%s connection state: error!", this->robotParamData.use_MDUI ? "MDUI" : "MDT");
    } else if(this->mdui_mdt_connection_state && !old_mdui_mdt_connection_state) {
        RCLCPP_INFO(this->get_logger(), "%s connection state: Ok", this->robotParamData.use_MDUI ? "MDUI" : "MDT");
    }
    old_mdui_mdt_connection_state = this->mdui_mdt_connection_state;
    ReceiveSerialData();
    this->RequestRobotStatus();
}

void MdRobotNode::InitMotorParameter() {
    switch(this->byCntInitStep) {
        case SETTING_PARAM_STEP_PID_PNT_VEL_CMD: {
            PID_PNT_VEL_CMD_t cmd_data;
            RCLCPP_INFO(this->get_logger(), "[SET] PID_PNT_VEL_CMD(PID NO: %d)", PID_PNT_VEL_CMD);
            cmd_data.enable_id1 = 1; cmd_data.rpm_id1 = 0; cmd_data.enable_id2 = 1; cmd_data.rpm_id2 = 0;
            cmd_data.req_monitor_id = this->robotParamData.use_MDUI ? REQUEST_PID_ROBOT_MONITOR : REQUEST_PNT_MAIN_DATA;
            pid_response_receive_count = 0; pid_request_cmd_vel_count = 1;
            PutMdData(PID_PNT_VEL_CMD, this->robotParamData.nRMID, (const uint8_t *)&cmd_data, sizeof(cmd_data));
            this->byCntInitStep = SETTING_PARAM_WAIT;
            break;
        }
        case SETTING_PARAM_WAIT: {
            if(pid_response_receive_count > 0) {
                pid_response_receive_count = 0;
                this->byCntInitStep = SETTING_PARAM_STEP_PID_ROBOT_PARAM;
            } else {
                this->check_connection_retry_count++;
                if(this->check_connection_retry_count >= MAX_CONNECTION_CHECK_COUNT) {
                    RCLCPP_ERROR(this->get_logger(), "!!! Error RS232(MDUI) or RS485(MDT) !!!");
                    this->fgInitsetting = INIT_SETTING_STATE_ERROR;
                    this->byCntInitStep = SETTING_PARAM_STEP_DONE;
                } else {
                    this->byCntInitStep = SETTING_PARAM_STEP_PID_PNT_VEL_CMD;
                }
            }
            break;
        }
        case SETTING_PARAM_STEP_PID_ROBOT_PARAM: {
            if(this->robotParamData.use_MDUI == 1) {
                PID_ROBOT_PARAM_t cmd_data;
                RCLCPP_INFO(this->get_logger(), "[SET] PID_ROBOT_PARAM(PID NO: %d)", PID_ROBOT_PARAM);
                cmd_data.nDiameter = (uint16_t)this->robotParamData.nDiameter;
                cmd_data.nWheelLength = (uint16_t)(this->robotParamData.nWheelLength * 1000.0);
                cmd_data.nGearRatio = (uint16_t)this->robotParamData.nGearRatio;
                PutMdData(PID_ROBOT_PARAM, MID_MDUI, (const uint8_t *)&cmd_data, sizeof(cmd_data));
            }
            this->byCntInitStep = SETTING_PARAM_STEP_PID_POSI_RESET;
            break;
        }
        case SETTING_PARAM_STEP_PID_POSI_RESET: {
            uint8_t dummy = 0;
            RCLCPP_INFO(this->get_logger(), "[SET] PID_POSI_RESET(PID NO: %d)", PID_POSI_RESET);
            PutMdData(PID_POSI_RESET, this->robotParamData.nRMID, &dummy, sizeof(dummy));
            this->byCntInitStep = SETTING_PARAM_STEP_PID_SLOW_START;
            break;
        }
        case SETTING_PARAM_STEP_PID_SLOW_START: {
            PID_SLOW_START_t cmd_data;
            RCLCPP_INFO(this->get_logger(), "[SET] PID_SLOW_START(PID NO: %d)", PID_SLOW_START);
            cmd_data.value = this->robotParamData.nSlowstart;
            PutMdData(PID_SLOW_START, this->robotParamData.nRMID, (const uint8_t *)&cmd_data, sizeof(cmd_data));
            this->byCntInitStep = SETTING_PARAM_STEP_PID_SLOW_DOWN;
            break;
        }
        case SETTING_PARAM_STEP_PID_SLOW_DOWN: {
            PID_SLOW_DOWN_t cmd_data;
            RCLCPP_INFO(this->get_logger(), "[SET] PID_SLOW_DOWN(PID NO: %d)", PID_SLOW_DOWN);
            cmd_data.value = this->robotParamData.nSlowdown;
            PutMdData(PID_SLOW_DOWN, this->robotParamData.nRMID, (const uint8_t *)&cmd_data, sizeof(cmd_data));
            this->byCntInitStep = SETTING_PARAM_STEP_PID_INV_SIGH_CMD;
            break;
        }
        case SETTING_PARAM_STEP_PID_INV_SIGH_CMD: {
            uint8_t cmd_data = (this->robotParamData.reverse_direction == 0) ? 1 : 0;
            RCLCPP_INFO(this->get_logger(), "[SET] PID_INV_SIGN_CMD(PID NO: %d)", PID_INV_SIGN_CMD);
            PutMdData(PID_INV_SIGN_CMD, this->robotParamData.nRMID, &cmd_data, 1);
            this->byCntInitStep = SETTING_PARAM_STEP_PID_INV_SIGH_CMD2;
            break;
        }
        case SETTING_PARAM_STEP_PID_INV_SIGH_CMD2: {
            uint8_t cmd_data = (this->robotParamData.reverse_direction == 0) ? 0 : 1;
            RCLCPP_INFO(this->get_logger(), "[SET] PID_INV_SIGN_CMD2(PID NO: %d)", PID_INV_SIGN_CMD2);
            PutMdData(PID_INV_SIGN_CMD2, this->robotParamData.nRMID, &cmd_data, 1);
            this->byCntInitStep = SETTING_PARAM_STEP_PID_USE_EPOSI;
            break;
        }
        case SETTING_PARAM_STEP_PID_USE_EPOSI: {
            uint8_t cmd_data = (this->robotParamData.enable_encoder == 0) ? 0 : 1;
            RCLCPP_INFO(this->get_logger(), "[SET] PID_USE_POSI(PID NO: %d)", PID_USE_POSI);
            PutMdData(PID_USE_POSI, this->robotParamData.nRMID, &cmd_data, 1);
            this->byCntInitStep = SETTING_PARAM_STEP_PID_PPR;
            break;
        }
        case SETTING_PARAM_STEP_PID_PPR: {
            if(this->robotParamData.use_MDUI == 1 && this->robotParamData.enable_encoder == 1) {
                PID_PPR_t cmd_data;
                RCLCPP_INFO(this->get_logger(), "[SET] PID_PPR(PID NO: %d)", PID_PPR);
                cmd_data.PPR = this->robotParamData.encoder_PPR;
                PutMdData(PID_PPR, this->robotParamData.nRMID, (const uint8_t *)&cmd_data, sizeof(cmd_data));
            }
            this->byCntInitStep = SETTING_PARAM_STEP_DONE;
            if(pid_request_cmd_vel_count == 2) {
                this->fgInitsetting = INIT_SETTING_STATE_OK;
            } else {
                this->fgInitsetting = INIT_SETTING_STATE_ERROR;
                RCLCPP_ERROR(this->get_logger(), "!!! Error %s interface !!!", this->robotParamData.use_MDUI ? "RS232" : "RS485");
            }
            break;
        }
        default: break;
    }
}

void MdRobotNode::RequestRobotStatus() {
    switch(this->byCntComStep) {
        case 0: {
            if(this->velCmdUpdateCount > 0) {
                this->velCmdUpdateCount = 0;
                PID_PNT_VEL_CMD_t pid_pnt_vel_cmd;
                int16_t *pGoalRPMSpeed = RobotSpeedToRPMSpeed(this->goal_cmd_speed, this->goal_cmd_ang_speed, this->robotParamData);
                RCLCPP_DEBUG(this->get_logger(), "Goal %.2f, %.2f, RPM L:%d, R:%d", this->goal_cmd_speed, this->goal_cmd_ang_speed, pGoalRPMSpeed[0], pGoalRPMSpeed[1]);
                pid_pnt_vel_cmd.enable_id1 = 1;
                pid_pnt_vel_cmd.rpm_id1 = this->mdui_mdt_connection_state ? pGoalRPMSpeed[0] : 0;
                pid_pnt_vel_cmd.enable_id2 = 1;
                pid_pnt_vel_cmd.rpm_id2 = this->mdui_mdt_connection_state ? pGoalRPMSpeed[1] : 0;
                pid_pnt_vel_cmd.req_monitor_id = this->robotParamData.use_MDUI ? REQUEST_PID_ROBOT_MONITOR : REQUEST_PNT_MAIN_DATA;
                PutMdData(PID_PNT_VEL_CMD, this->robotParamData.nRMID, (const uint8_t *)&pid_pnt_vel_cmd, sizeof(pid_pnt_vel_cmd));
                pid_request_cmd_vel_count++;
            }
            if(this->robotParamData.use_MDUI == 1 && curr_pid_robot_monitor.byPlatStatus.bits.bEmerSW == 1) {
                this->byCntComStep = 3;
            } else if(this->reset_pos_flag || this->reset_alarm_flag) {
                this->byCntComStep = 3;
            } else {
                this->byCntComStep = 4;
            }
            break;
        }
        case 3: {
            if(this->robotParamData.use_MDUI == 1 && curr_pid_robot_monitor.byPlatStatus.bits.bEmerSW == 1) {
                PID_PNT_TQ_OFF_t pid_pnt_tq_off;
                pid_pnt_tq_off.enable_id1 = 1; pid_pnt_tq_off.enable_id2 = 1;
                pid_pnt_tq_off.req_monitor_id = REQUEST_PNT_MAIN_DATA;
                PutMdData(PID_PNT_TQ_OFF, this->robotParamData.nRMID, (const uint8_t *)&pid_pnt_tq_off, sizeof(pid_pnt_tq_off));
            }
            if(this->reset_pos_flag) {
                uint8_t dummy = 0;  
                this->reset_pos_flag = false;
                PutMdData(PID_POSI_RESET, this->robotParamData.nRMID, &dummy, sizeof(dummy));
            } else if(this->reset_alarm_flag) {
                uint8_t cmd_pid = CMD_ALARM_RESET;
                this->reset_alarm_flag = false;
                PutMdData(PID_COMMAND, this->robotParamData.nRMID, &cmd_pid, 1);
            }
            this->byCntComStep = 0;
            break;
        }
        case 4: {
            uint8_t request_pid = this->robotParamData.use_MDUI ? PID_ROBOT_MONITOR2 : PID_IO_MONITOR;
            RCLCPP_DEBUG(this->get_logger(), "REQ: %s", this->robotParamData.use_MDUI ? "PID_ROBOT_MONITOR2" : "PID_IO_MONITOR");
            PutMdData(PID_REQ_PID_DATA, this->robotParamData.nRMID, &request_pid, 1);
            this->byCntComStep = 0;
            break;
        }
        default:
            this->byCntComStep = 0;
            break;
    }
}