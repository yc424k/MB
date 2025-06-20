#include "md_robot_node/com.hpp"
#include "md_robot_node/main.hpp"
#include "md_robot_node/robot.hpp"
#include "serial_driver/serial_port.hpp"
#include <vector>
#include <string>
#include <memory>
#include <iostream> // fprintf를 사용하기 위해 추가

// main.cpp에 정의된 전역 노드 포인터를 참조
extern std::shared_ptr<MdRobotNode> g_node_ptr;
extern volatile uint32_t pid_request_cmd_vel_count;

// 이 파일에서 상태를 관리하는 변수들
PID_PNT_MAIN_DATA_t curr_pid_pnt_main_data;
PID_IO_MONITOR_t curr_pid_io_monitor;
PID_ROBOT_MONITOR_t curr_pid_robot_monitor;
PID_ROBOT_MONITOR2_t curr_pid_robot_monitor2;
uint32_t pid_response_receive_count = 0;

// serial_driver 관련 객체들
std::unique_ptr<drivers::common::IoContext> owned_ctx;
std::unique_ptr<drivers::serial_driver::SerialPort> ser_port;

// 통신 프로토콜용 버퍼
uint8_t serial_comm_rcv_buff[MAX_PACKET_SIZE];
uint8_t serial_comm_snd_buff[MAX_PACKET_SIZE];

int InitSerialComm(const std::string& port_name, int baud_rate)
{
    if (!g_node_ptr) return -1;
    auto logger = g_node_ptr->get_logger();
    RCLCPP_INFO(logger, "Attempting to open serial port: %s at %d bps", port_name.c_str(), baud_rate);

    owned_ctx = std::make_unique<drivers::common::IoContext>(1);
    
    drivers::serial_driver::SerialPortConfig port_config(baud_rate,
        drivers::serial_driver::FlowControl::NONE,
        drivers::serial_driver::Parity::NONE,
        drivers::serial_driver::StopBits::ONE);

    // open() 호출을 제거하고, 생성자에서 모든 것을 처리하도록 시도합니다.
    try {
        ser_port = std::make_unique<drivers::serial_driver::SerialPort>(*owned_ctx, port_name, port_config);
        // ser_port->open(); // 이 라인을 제거합니다.
    } catch (const std::exception & e) {
        RCLCPP_ERROR(logger, "Failed during SerialPort object creation: %s", e.what());
        return -1;
    }

    // 객체 생성 후, 포트가 열렸는지 바로 확인합니다.
    if (ser_port && ser_port->is_open()) {
        RCLCPP_INFO(logger, "Serial port opened successfully.");
        return 1;
    } else {
        RCLCPP_ERROR(logger, "Serial port not open after initialization attempt.");
        // 실패의 원인이 될 수 있는 예외를 던져서 노드를 종료시킵니다.
        throw std::runtime_error("Failed to initialize serial communication.");
    }
}

uint8_t CalCheckSum(uint8_t *pData, uint16_t length)
{
    uint16_t sum = 0;
    for(int i = 0; i < length; i++) {
        sum += pData[i];
    }
    sum = ~sum + 1;
    return (uint8_t)sum;
}

int PutMdData(uint8_t pid, uint16_t rmid, const uint8_t *pData, uint16_t length) {
    if (!g_node_ptr || !ser_port) return -1;
    
    if (ser_port->is_open()) {
        uint16_t len = 0;
        serial_comm_snd_buff[len++] = rmid;
        serial_comm_snd_buff[len++] = g_node_ptr->robotParamData.nIDPC;
        serial_comm_snd_buff[len++] = 1;
        serial_comm_snd_buff[len++] = pid;
        serial_comm_snd_buff[len++] = length;
        memcpy(&serial_comm_snd_buff[len], pData, length);
        len += length;
        serial_comm_snd_buff[len++] = CalCheckSum(serial_comm_snd_buff, len);

        try {
            std::vector<uint8_t> data_to_send(serial_comm_snd_buff, serial_comm_snd_buff + len);
            ser_port->send(data_to_send);
            return 1;
        } catch (const std::exception& e) {
            RCLCPP_WARN(g_node_ptr->get_logger(), "Error writing to serial port: %s", e.what());
        }
    }
    return -1;
}

int MdReceiveProc()
{
    if (!g_node_ptr) return -1;
    auto logger = g_node_ptr->get_logger();
    
    uint8_t byRcvPID = serial_comm_rcv_buff[3];
    uint8_t byRcvDataSize = serial_comm_rcv_buff[4];
    uint8_t *pRcvData = &serial_comm_rcv_buff[5];

    switch(byRcvPID)
    {
        case PID_IO_MONITOR:
            if(byRcvDataSize <= sizeof(PID_IO_MONITOR_t)) {
                RCLCPP_DEBUG(logger, "RCV: PID_IO_MONITOR");
                memcpy(&curr_pid_io_monitor, pRcvData, byRcvDataSize);
            }
            break;
        case PID_ROBOT_MONITOR2:
            if(byRcvDataSize <= sizeof(PID_ROBOT_MONITOR2_t)) {
                RCLCPP_DEBUG(logger, "RCV: PID_ROBOT_MONITOR2");
                memcpy(&curr_pid_robot_monitor2, pRcvData, byRcvDataSize);
            }
            break;
        case PID_PNT_MAIN_DATA:
            if(byRcvDataSize == sizeof(PID_PNT_MAIN_DATA_t)) {
                pid_response_receive_count++;
                pid_request_cmd_vel_count = 2;
                memcpy(&curr_pid_pnt_main_data, pRcvData, sizeof(PID_PNT_MAIN_DATA_t));
                auto msg = MakeMDRobotMessage1(&curr_pid_pnt_main_data, curr_pid_io_monitor, g_node_ptr->get_clock(), g_node_ptr->get_logger());
                g_node_ptr->md_robot_message1_pub->publish(msg);
            }
            break;
        case PID_ROBOT_MONITOR:
            if(byRcvDataSize == sizeof(PID_ROBOT_MONITOR_t)) {
                pid_response_receive_count++;
                pid_request_cmd_vel_count = 2;
                memcpy(&curr_pid_robot_monitor, pRcvData, sizeof(PID_ROBOT_MONITOR_t));
                if (g_node_ptr->robotParamData.use_MDUI == 1) {
                    auto msg = MakeMDRobotMessage2(&curr_pid_robot_monitor, curr_pid_robot_monitor2, g_node_ptr->robotParamData, g_node_ptr->get_clock(), g_node_ptr->get_logger());
                    g_node_ptr->md_robot_message2_pub->publish(msg);
                }
            }
            break;
        case PID_ROBOT_PARAM:
            if(byRcvDataSize == sizeof(PID_ROBOT_PARAM_t)) {
                 RCLCPP_DEBUG(logger, "RCV: PID_ROBOT_PARAM");
            }
            break;
        default:
            break;
    }
    return 1;
}

int AnalyzeReceivedData(const std::vector<uint8_t>& buffer) { 
    if(!g_node_ptr) return 0;

    static uint32_t rcv_step = 0, byPacketNum = 0;
    static uint8_t byChkSum = 0;
    static uint16_t byMaxDataNum = 0, byDataNum = 0;
    
    for (uint8_t data : buffer)
    {
        switch(rcv_step)
        {
            case 0:
                if(data == g_node_ptr->robotParamData.nIDPC) {
                    byPacketNum = 0; byChkSum = data; serial_comm_rcv_buff[byPacketNum++] = data; rcv_step++;
                }
                break;
            case 1:
                if(data == g_node_ptr->robotParamData.nIDMDUI || data == g_node_ptr->robotParamData.nIDMDT) {
                    byChkSum += data; serial_comm_rcv_buff[byPacketNum++] = data; rcv_step++;
                } else { rcv_step = 0; }
                break;
            case 2:
                if(data == 1 || data == ID_ALL) {
                    byChkSum += data; serial_comm_rcv_buff[byPacketNum++] = data; rcv_step++;
                } else { rcv_step = 0; }
                break;
            case 3:
                byChkSum += data; serial_comm_rcv_buff[byPacketNum++] = data; rcv_step++;
                break;
            case 4:
                byChkSum += data; serial_comm_rcv_buff[byPacketNum++] = data; byMaxDataNum = data; byDataNum = 0; rcv_step++;
                break;
            case 5:
                byChkSum += data; serial_comm_rcv_buff[byPacketNum++] = data;
                if(++byDataNum >= MAX_DATA_SIZE) { rcv_step = 0; break; }
                if(byDataNum >= byMaxDataNum) { rcv_step++; }
                break;
            case 6:
                byChkSum += data; serial_comm_rcv_buff[byPacketNum++] = data;
                if(byChkSum == 0) {
                    MdReceiveProc();
                } else {
                    RCLCPP_WARN(g_node_ptr->get_logger(), "Error.Checksum");
                }
                rcv_step = 0;
                break;
            default:
                rcv_step = 0;
                break;
        }
    }
    return 1;
}

void ReceiveSerialData() {
    if (ser_port && ser_port->is_open()) {
        std::vector<uint8_t> buffer;
        try {
            ser_port->receive(buffer);
            if (!buffer.empty()) {
                AnalyzeReceivedData(buffer);
            }
        } catch (const std::exception& e) {
            if (g_node_ptr) {
                RCLCPP_WARN(g_node_ptr->get_logger(), "Error reading from serial port: %s", e.what());
            }
        }
    }
}