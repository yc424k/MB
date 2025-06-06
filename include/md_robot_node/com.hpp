#pragma once

#include <cstdint>
#include <string>

#define ID_ALL 0xfe
#define MAX_PACKET_SIZE 128
#define MAX_DATA_SIZE 21
#define REQUEST_PNT_MAIN_DATA 2
#define REQUEST_PID_ROBOT_MONITOR 3
#define CMD_ALARM_RESET 8

#define MID_MDUI 184
#define MID_MDT 183
#define MID_REMOTE_PC 172

typedef enum
{
    PID_REQ_PID_DATA = 4,
    PID_COMMAND = 10,
    PID_POSI_RESET = 13,
    PID_INV_SIGN_CMD = 16,
    PID_INV_SIGN_CMD2 = 18,
    PID_USE_POSI = 46,
    PID_PPR = 126,
    PID_SLOW_START = 153,
    PID_SLOW_DOWN = 154,
    PID_PNT_TQ_OFF = 174,
    PID_IO_MONITOR = 194,
    PID_GAIN = 203,
    PID_PNT_VEL_CMD = 207,
    PID_PNT_MAIN_DATA = 210,
    PID_ROBOT_MONITOR2 = 224,
    PID_ROBOT_PARAM = 247,
    PID_ROBOT_MONITOR = 253,
} PID_CMD_t;

typedef enum
{
    SETTING_PARAM_STEP_PID_PNT_VEL_CMD = 1,
    SETTING_PARAM_STEP_PID_ROBOT_PARAM,
    SETTING_PARAM_WAIT,
    SETTING_PARAM_STEP_PID_POSI_RESET,
    SETTING_PARAM_STEP_PID_SLOW_START,
    SETTING_PARAM_STEP_PID_SLOW_DOWN,
    SETTING_PARAM_STEP_PID_GAIN,
    SETTING_PARAM_STEP_PID_INV_SIGH_CMD,
    SETTING_PARAM_STEP_PID_INV_SIGH_CMD2,
    SETTING_PARAM_STEP_PID_USE_EPOSI,
    SETTING_PARAM_STEP_PID_PPR,
    SETTING_PARAM_STEP_DONE,
} SETTINNG_PARAM_STEP_t;

typedef enum
{
    INIT_SETTING_STATE_NONE = 0,
    INIT_SETTING_STATE_OK = 1,
    INIT_SETTING_STATE_ERROR = 2,
} INIT_SETTING_STATE_t;

typedef struct
{
    int16_t PPR;
} __attribute__((aligned(1), packed)) PID_PPR_t;

typedef struct
{
    uint8_t enable_id1;
    int16_t rpm_id1;
    uint8_t enable_id2;
    int16_t rpm_id2;
    uint8_t req_monitor_id;
} __attribute__((aligned(1), packed)) PID_PNT_VEL_CMD_t;

typedef struct
{
    uint16_t nDiameter;
    uint16_t nWheelLength;
    uint16_t nGearRatio;
} __attribute__((aligned(1), packed)) PID_ROBOT_PARAM_t;

typedef struct
{
    uint8_t enable_id1;
    uint8_t enable_id2;
    uint8_t req_monitor_id;
} __attribute__((aligned(1), packed)) PID_PNT_TQ_OFF_t;

typedef union _MOTOR_STATE_t
{
    uint8_t val;
    struct
    {
        uint8_t Alarm : 1;
        uint8_t CtrlFail : 1;
        uint8_t OverVolt : 1;
        uint8_t OverTemp : 1;
        uint8_t OverLoad : 1;
        uint8_t HallFail : 1;
        uint8_t InvVel : 1;
        uint8_t Stall : 1;
    } bits;
} __attribute__((aligned(1), packed)) MOTOR_STATE_t;

typedef struct
{
    int16_t rpm_id1;
    int16_t current_id1;
    MOTOR_STATE_t mtr_state_id1;
    int32_t mtr_pos_id1;
    int16_t rpm_id2;
    int16_t current_id2;
    MOTOR_STATE_t mtr_state_id2;
    int32_t mtr_pos_id2;
} __attribute__((aligned(1), packed)) PID_PNT_MAIN_DATA_t;

typedef struct
{
    int16_t rpm_id1;
    int16_t current_id1;
    MOTOR_STATE_t mtr_state_id1;
    uint8_t input_signal;
    int16_t speed_volume;
    uint8_t dip_sw_info;
    uint8_t hall_sensor;
    uint8_t status_2;
    uint8_t input_switch_sigal;
    int16_t input_voltage;
    uint8_t slow_start;
    uint8_t slow_stop;
    uint8_t load;
} __attribute__((aligned(1), packed)) PID_IO_MONITOR_t;

typedef union _PLATFORM_STATE_t
{
    uint8_t val;
    struct
    {
        uint8_t bEmerSW : 1;
        uint8_t bBusy : 1;
        uint8_t bBumper1 : 1;
        uint8_t bBumper2 : 1;
        uint8_t bAlarm1 : 1;
        uint8_t balarm2 : 1;
        uint8_t bChargerState : 1;
        uint8_t bRobotTurnOrSync : 1;
    } bits;
} __attribute__((aligned(1), packed)) PLATFORM_STATE_t;

typedef struct
{
    uint32_t lTempPosi_x;
    uint32_t lTempPosi_y;
    uint16_t sTempTheta;
    uint8_t battery_percent;
    uint8_t byUS1;
    uint8_t byUS2;
    uint8_t byUS3;
    uint8_t byUS4;
    PLATFORM_STATE_t byPlatStatus;
    int16_t linear_velocity;
    int16_t angular_velocity;
} __attribute__((aligned(1), packed)) PID_ROBOT_MONITOR_t;

typedef struct
{
    uint16_t sVoltIn;
    // ...
} __attribute__((aligned(1), packed)) PID_ROBOT_MONITOR2_t;

typedef struct
{
    uint16_t value;
} PID_SLOW_START_t;

typedef struct
{
    uint16_t value;
} PID_SLOW_DOWN_t;

typedef struct
{
    int use_MDUI;
    int nIDPC;
    int nIDMDUI;
    int nIDMDT;
    int nBaudrate;
    int nDiameter;
    double wheel_radius;
    double nWheelLength;
    int nRMID;
    int nGearRatio;
    int reverse_direction;
    int enable_encoder;
    int encoder_PPR;
    int nMaxRPM;
    int nSlowstart;
    int nSlowdown;
} ROBOT_PARAMETER_t;

// 함수 원형 선언부
int InitSerialComm(const std::string &port_name, int baud_rate);
int PutMdData(uint8_t pid, uint16_t rmid, const uint8_t *pData, uint16_t length);
void ReceiveSerialData(void);
int AnalyzeReceivedData(const std::vector<uint8_t> &buffer);
uint8_t CalCheckSum(uint8_t *pData, uint16_t length);
int MdReceiveProc(void);