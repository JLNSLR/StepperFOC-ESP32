#ifndef DRV_CAN_INTERFACE_H
#define DRV_CAN_INTERFACE_H

//#include <joint_control_global_def.h>

#include <CAN.h>
#include <drv_can_utils.h>


class CANInterface {
public:
    CANInterface();
    CANInterface(uint8_t joint_id);

    void init();

    // Output Commands
    void send_drive_motion_data();
    void send_drive_torque_data();
    void send_drive_controllersys_state();
    //Process received commands
    void process_motion_command(drvComm_MotionCmd motion_cmd);
    void process_drive_sys_controller_command(drvComm_controllerCmd controller_cmd);
    void process_drive_sys_parameter_command(drvComm_paramsCmd params_cmd);
    void process_update_PID_command(drvComm_PID_update pid_update);
    void process_light_command(drvComm_LightCmd light_cmd);

    void onReceive(int packetSize);

    uint8_t joint_id = 0;
    bool running = false;

    TaskHandle_t can_task_th;

private:
    static void handle_CAN_task(void* params);


};

#endif // !DRV_CAN_INTERFACE