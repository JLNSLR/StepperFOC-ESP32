#ifndef DRV_CAN_UTILS
#define DRV_CAN_UTILS

#include <drive_system.h>

#define MAX_VEL_DATA_DEG_PER_S 400 // (°/s)
#define MAX_POS_DATA_DEG 180 // °
#define MAX_ACC_DATA_DEG 2000
#define MAX_TORQUE_DATA_NM 150

#define MAX_POS_FACTOR  0.00549316
#define MAX_POS_FACTOR_PACK  182.044444
#define MAX_VEL_FACTOR  0.012207
#define MAX_VEL_FACTOR_PACK  81.92
#define MAX_ACC_FACTOR  0.0610351
#define MAX_ACC_FACTOR_PACK  16.384

#define MAX_JTORQUE_FACTOR 0.0001430511
#define MAX_JTORQUE_FACTOR_PACK  6990.50667

#define MAX_PID_GAIN_VAL 1000000
#define PID_GAIN_VAL_FACTOR 0.000023283064365386962890625
#define PID_GAIN_VAL_FACTOR_PACK 42949.67296

#define MAX_MOTOR_TORQUE_VAL 100.0
#define MOTOR_TORQUE_FACTOR 0.000095367431640625
#define MOTOR_TORQUE_FACTOR_PACK 10485.76


float motor_torque_factor;
float motor_torque_factor_pack;
float max_motor_torque;



/* CAN-ID used to identify origin and type of a CAN message.
11 bit id: 0x000 - 0x7ff
first three bits  - robot sys id -> default value: 000 - 0x0
bits 3 - 6: robot joint id -> values 0x0 -> 0x6
bits 7-10: message type: values: 0x0 -> 0xF

*/

/* message type id: 4 bit value 0x0 -> 0xF */
enum drv_can_msg_type_id {
    drive_motion_state = 0x0, drive_torque = 0x1, drive_state = 0x2,
    drive_motion_command = 0x3, drive_sys_controller_command = 0x4, drive_sys_param_command = 0x5,
    drive_sys_light_command = 0x6, drive_sys_pid_command = 0x7
};

enum drv_can_motionCmdType { traj_command, direct_command, goto_command };


struct drvComm_CANID {
    union {
        uint16_t msg_id; //11bit id 0x7FF
        struct {
            int sys_id : 3;
            int joint_id : 4;
            drv_can_msg_type_id msg_type : 4;
        };
    };
};


struct drvComm_controllerCmd {
    bool start : 1;
    bool stop : 1;
    drvSys_controlMode mode : 8;
};


enum drvComm_parameterType { torque_limit, pos_limit_high, pos_limit_low, max_vel, max_current_mA };

struct drvComm_paramsCmd {
    drvComm_parameterType type : 8;
    int parameter : 32;
};


struct drvComm_MotionState {
    int pos : 16;
    int vel : 16;
    int acc : 16;
    int m_torque : 9;
};

struct drvComm_TorqueState {
    int torque_val : 24;
};

struct drvComm_ControllerState {
    int stateFlag : 3;
    int active : 1;
    int error : 1;
    int mode : 3;
    int calibrated : 1;
    int temperature : 12;
    int overtemperature : 1;
    int max_motor_torque : 20;
};

struct drvComm_LightCmd {
    int rgb_hsv : 1;
    int rh : 8;
    int gs : 8;
    int bv : 8;
};

struct drvComm_PID_update {
    int PID_K : 3;
    int PID_t : 3;
    int gain : 32;
    bool save : 1;
};


/* Motion command Types:
    0 - new targets (pos, vel_ff, torque_ff)
    1 - go to new pos (interpolate to pos)
    2 - direct mode
    3 - new targets hybrid (pos, vel_ff, torque_ff, joint_torque)
*/

struct drvComm_MotionCmd_traj_target {
    int pos : 16;
    int vel_ff : 16;
    int torque_ff : 9;
};

struct drvComm_MotionCmd_goTo_target {
    int pos : 16;
    int vel_max : 16;
};

struct drvComm_MotionCmd_direct_motor_torque_target {
    int m_torque_target : 9;
};



struct drvComm_MotionCmd {
    drv_can_motionCmdType type : 3;

    uint8_t drvComm_MotionData;
    union {
        drvComm_MotionCmd_traj_target traj_target;
        drvComm_MotionCmd_goTo_target goTo_target;
        drvComm_MotionCmd_direct_motor_torque_target direct_target;
    };

};


struct drvComm_CAN_motionMsg {

    union {
        uint8_t bytes[8];
        drvComm_MotionState motionMsg;
    };
};

struct drvComm_CAN_torqueMsg {
    union {
        uint8_t bytes[3];
        drvComm_TorqueState torque;
    };
};


struct drvComm_motion_data {
    float pos_deg;
    float vel_deg;
    float acc_deg;
    float motor_torque;
};

drvComm_MotionCmd drvComm_gen_motion_command(drv_can_motionCmdType type, float pos = 0, float vel = 0, float torque = 0);

drvComm_controllerCmd drvComm_gen_controller_command(bool start = true, bool stop = false, drvSys_controlMode control_mode=cascade_position_control );

drvComm_PID_update drvComm_gen_pid_update(int controller_type, int gain_type, float gain_value);

drvComm_paramsCmd drvComm_gen_params_command(drvComm_parameterType type, float param_value);

drvComm_motion_data drvComm_unpack_motion_data(drvComm_MotionState motion_state);

float drvComm_unpack_torque_sensor_data(drvComm_TorqueState torque_state);


#endif // ! DRV_CAN_UTILS
