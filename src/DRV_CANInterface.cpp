#include <DRV_CANINterface.h>
#include <joint_control_global_def.h>
#include <FreeRTOS.h>

CANInterface::CANInterface() {
    CAN.setPins(CAN_RX, CAN_TX);
    if (!CAN.begin(1000E3)) {
        Serial.println("Starting CAN failed!");
        running = true;
    }
    else {
        Serial.println("Starting CAN successful!");
    }

    drvSys_parameters drive_param = drvSys_get_parameters();

    max_motor_torque = drive_param.max_torque_Nm;
    motor_torque_factor = double(drive_param.max_torque_Nm) / double(255.0);
    motor_torque_factor_pack = double(255) / double(drive_param.max_torque_Nm);
};


CANInterface::CANInterface(uint8_t joint_id) {
    this->joint_id = joint_id;
};

void CANInterface::init() {
    if (!running) {
        CAN.setPins(CAN_RX, CAN_TX);
        if (!CAN.begin(1000E3)) {
            Serial.println("Starting CAN failed!");
            running = true;
        }
        else {
            Serial.println("Starting CAN successful!");
        }
    }

    // Set up CAN TASK

    xTaskCreatePinnedToCore(
        this->handle_CAN_task,
        "CAN_Task",
        2000,
        this,
        7,
        &can_task_th,
        1
    );


}


void CANInterface::send_drive_motion_data() {
    drvSys_driveState data = drvSys_get_drive_state();

    drvComm_MotionState motion_data;

    int16_t pos = data.joint_pos * MAX_POS_FACTOR_PACK;
    int16_t vel = data.joint_vel * MAX_VEL_FACTOR_PACK;
    int16_t acc = data.joint_acc * MAX_ACC_FACTOR_PACK;

    int16_t m_torque = data.joint_torque * motor_torque_factor_pack;

    drvComm_CAN_motionMsg msg;
    msg.motionMsg = motion_data;

    motion_data.acc = acc;
    motion_data.pos = pos;
    motion_data.vel = vel;
    motion_data.m_torque = m_torque;

    drvComm_CANID msg_id;
    msg_id.sys_id = 0;
    msg_id.joint_id = joint_id;
    msg_id.msg_type = drive_motion_state;

    //xSemaphoreTake(mutex)
    CAN.beginPacket(msg_id.msg_id);
    CAN.write(msg.bytes, 8);
    CAN.endPacket();

}

void CANInterface::send_drive_torque_data() {
    drvSys_driveState data = drvSys_get_drive_state();

    drvComm_CAN_torqueMsg msg;
    msg.torque.torque_val = data.joint_torque * MAX_JTORQUE_FACTOR_PACK;

    drvComm_CANID msg_id;
    msg_id.sys_id = 0;
    msg_id.joint_id = joint_id;
    msg_id.msg_type = drive_torque;

    CAN.beginPacket(msg_id.msg_id);
    CAN.write(msg.bytes, 3);
    CAN.endPacket();
}

void CANInterface::send_drive_controllersys_state() {
    drvSys_controllerState ctrl_state_drive = drvSys_get_controllerState();

    drvComm_ControllerState ctrl_state_data;

    ctrl_state_data.mode = ctrl_state_drive.control_mode;
    ctrl_state_data.calibrated = ctrl_state_drive.calibrated;
    ctrl_state_data.stateFlag = ctrl_state_drive.state_flag;
    ctrl_state_data.overtemperature = ctrl_state_drive.overtemperature;

    ctrl_state_data.temperature = ctrl_state_drive.temperature; //add conversion factor

    if (ctrl_state_drive.state_flag == control_active) {
        ctrl_state_data.active = true;
    }
    else {
        ctrl_state_data.active = false;
    }
    if (ctrl_state_data.stateFlag == error) {
        ctrl_state_data.error = true;
    }
    else {
        ctrl_state_data.active = false;
    }

    //obtain max torque

    drvSys_parameters drive_params = drvSys_get_parameters();
    ctrl_state_data.max_motor_torque = drive_params.max_torque_Nm * MOTOR_TORQUE_FACTOR_PACK;

    drvComm_CANID msg_id;
    msg_id.joint_id = joint_id;
    msg_id.msg_type = drive_state;
    msg_id.sys_id = 0;

    union {
        uint8_t bytes[7];
        drvComm_ControllerState state_data;
    }state_can_msg;

    state_can_msg.state_data = ctrl_state_data;

    CAN.beginPacket(msg_id.msg_id);
    CAN.write(state_can_msg.bytes, 7);
    CAN.endPacket();


}


void CANInterface::process_motion_command(drvComm_MotionCmd motion_cmd) {

    if (motion_cmd.type == traj_command) {

        if (drvSys_mode == cascade_position_control && drvSys_mode == admittance_control) {
            float target_pos = float(motion_cmd.traj_target.pos) * MAX_POS_FACTOR;
            float vel_ff = float(motion_cmd.traj_target.vel_ff) * MAX_VEL_FACTOR;
            float torque_ff = float(motion_cmd.traj_target.torque_ff) * motor_torque_factor;


            drvSys_set_feed_forward_torque(torque_ff);
            drvSys_set_feed_forward_velocity(vel_ff);
            drvSys_set_target_pos(target_pos);
        }
    }

    if (motion_cmd.type = direct_command) {
        if (drvSys_mode == direct_torque) {
            float target_torque = float(motion_cmd.direct_target.m_torque_target) * motor_torque_factor;

            drvSys_set_target_torque(target_torque);
        }
    }

    if (motion_cmd.type == goto_command) {
        // TO DO
    }

}

void CANInterface::process_update_PID_command(drvComm_PID_update pid_update) {

    // Collect old PID Values

    drvSys_parameters params = drvSys_get_parameters();

    if (pid_update.PID_t == 0) {
        // Position Controller

        float P = params.pos_pid_gains.K_p;
        float I = params.pos_pid_gains.K_i;
        float D = params.pos_pid_gains.K_d;

        if (pid_update.PID_K == 0) {
            P = pid_update.gain * PID_GAIN_VAL_FACTOR;
        }
        if (pid_update.PID_K == 1) {
            I = pid_update.gain * PID_GAIN_VAL_FACTOR;
        }
        if (pid_update.PID_K == 2) {
            D = pid_update.gain * PID_GAIN_VAL_FACTOR;
        }

        bool save = pid_update.save;

        drvSys_set_pos_PID_gains(P, I, D, save);

        return;
    }
    if (pid_update.PID_t == 1) {
        // Velocity Controller

        float P = params.vel_pid_gains.K_p;
        float I = params.vel_pid_gains.K_i;
        float D = params.vel_pid_gains.K_d;

        if (pid_update.PID_K == 0) {
            P = pid_update.gain * PID_GAIN_VAL_FACTOR;
        }
        if (pid_update.PID_K == 1) {
            I = pid_update.gain * PID_GAIN_VAL_FACTOR;
        }
        if (pid_update.PID_K == 2) {
            D = pid_update.gain * PID_GAIN_VAL_FACTOR;
        }

        bool save = pid_update.save;

        drvSys_set_vel_PID_gains(P, I, D, save);

        return;
    }
    if (pid_update.PID_t == 2) {
        // Admittance Controller

        float spring = params.admittance_gains.virtual_spring;
        float damper = params.admittance_gains.virtual_damping;
        float inertia = params.admittance_gains.virtual_inertia;

        if (pid_update.PID_K == 0) {
            spring = pid_update.gain * PID_GAIN_VAL_FACTOR;
        }
        if (pid_update.PID_K == 1) {
            damper = pid_update.gain * PID_GAIN_VAL_FACTOR;
        }
        if (pid_update.PID_K == 2) {
            inertia = pid_update.gain * PID_GAIN_VAL_FACTOR;
        }

        bool save = pid_update.save;

        drvSys_set_admittance_params(spring, damper, inertia, save);

        return;
    }
}

void CANInterface::onReceive(int packetSize) {

    drvComm_CANID can_id;
    can_id.msg_id = CAN.packetId();

    union {
        uint8_t array[8];
        uint64_t _int;
    }paket;

    paket._int = CAN.read();

    if (can_id.msg_id == drive_motion_command) {

        union {
            uint64_t _int;
            drvComm_MotionCmd mot_cmd;
        }motion_input;

        motion_input._int = paket._int;


        this->process_motion_command(motion_input.mot_cmd);
        return;
    }

    if (can_id.msg_id == drive_sys_controller_command) {


        union {
            uint64_t _int;
            drvComm_controllerCmd cmd;
        }input;


        this->process_drive_sys_controller_command(input.cmd);
        return;
    }
    if (can_id.msg_id == drive_sys_param_command) {

        union {
            uint64_t _int;
            drvComm_paramsCmd cmd;
        }input;

        this->process_drive_sys_parameter_command(input.cmd);
        return;
    }
    if (can_id.msg_id == drive_sys_pid_command) {

        union {
            uint64_t _int;
            drvComm_PID_update cmd;
        }input;

        this->process_update_PID_command(input.cmd);
        return;
    }
    if (can_id.msg_id == drive_sys_light_command) {
        //
        return;
    }

}

void process_drive_sys_controller_command(drvComm_controllerCmd controller_cmd) {

    if (controller_cmd.stop == true) {
        drvSys_stop_controllers();
        return;
    }
    if (controller_cmd.start == true) {

        drvSys_controlMode mode = controller_cmd.mode;

        drvSys_start_motion_control(mode);
        return;
    }
}
void process_drive_sys_parameter_command(drvComm_paramsCmd params_cmd) {

    if (params_cmd.type == torque_limit) {
        drvSys_parameter_config.max_torque_Nm = params_cmd.parameter;
        return;
    }
    if (params_cmd.type == pos_limit_high) {
        drvSys_parameter_config.limit_high_deg = params_cmd.parameter;
        return;
    }
    if (params_cmd.type == pos_limit_low) {
        drvSys_parameter_config.limit_low_deg = params_cmd.parameter;
        return;
    }
    if (params_cmd.type == max_vel) {
        drvSys_parameter_config.max_vel = params_cmd.parameter;
        return;
    }
    if (params_cmd.type == max_current_mA) {
        drvSys_parameter_config.max_current_mA = params_cmd.parameter;
        return;
    }

}



//void 