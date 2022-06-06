#include <joint_control_asci_interface.h>

TaskHandle_t asci_interface_task;

bool jCtrl_active = false;
bool jCtrl_state_output = false;
bool jCtrl_update_output = false;

struct {
    bool pos = true;
    bool vel = false;
    bool acc = false;
    bool m_torque = false;
    bool joint_torque = false;
    bool pos_target = false;
    bool vel_target = false;
    bool m_torque_target = false;
}jCtrl_state_output_config;

void jCtrl_asci_start_interface() {

    jCtrl_active = true;

    /* Create Task */

    xTaskCreatePinnedToCore(
        _jCtrl_asci_task,   // function name
        "jCtrl_asci_task", // task name
        5000,      // Stack size (bytes)
        NULL,      // task parameters
        2,         // task priority
        &asci_interface_task,
        1 // task handle
    );


};

void _jCtrl_asci_task(void* params) {

    const TickType_t asci_delay = jCtrl_asci_interface_period_ms / portTICK_PERIOD_MS;

    while (jCtrl_active) {

        xSemaphoreTake(glob_Serial_mutex, portMAX_DELAY);
        size_t available_bytes = Serial.available();
        if (available_bytes > 0) {
            char* input_buffer;
            String input = Serial.readString();



            _jCtrl_parse_asci_command(input);
            xSemaphoreGive(glob_Serial_mutex);

        }
        xSemaphoreGive(glob_Serial_mutex);

        if (jCtrl_state_output) {
            _jCtrl_asci_handle_debug_output();
        }

        vTaskDelay(asci_delay);

    }

};

void _jCtrl_asci_handle_debug_output() {


    xSemaphoreTake(glob_Serial_mutex, portMAX_DELAY);
    if (jCtrl_update_output) {
        // Output Legends
        bool prev_output = false;
        if (jCtrl_state_output_config.pos) {
            Serial.print("pos:");
            prev_output = true;
        };
        if (jCtrl_state_output_config.vel) {
            if (prev_output) {
                Serial.print(',');
            }
            Serial.print("vel:");
        };
        if (jCtrl_state_output_config.acc) {
            if (prev_output) {
                Serial.print(',');
            }
            Serial.print("acc:");
        };
        if (jCtrl_state_output_config.m_torque) {
            if (prev_output) {
                Serial.print(',');
            }
            Serial.print("M_torque:");
        }
        if (jCtrl_state_output_config.joint_torque) {
            if (prev_output) {
                Serial.print(',');
            }
            Serial.print("joint_torque:");
        }
        if (jCtrl_state_output_config.pos_target) {
            if (prev_output) {
                Serial.print(',');
            }
            Serial.print("pos_ref:");
        }
        if (jCtrl_state_output_config.vel_target) {
            if (prev_output) {
                Serial.print(',');
            }
            Serial.print("vel_ref:");
        }
        if (jCtrl_state_output_config.m_torque_target) {
            if (prev_output) {
                Serial.print(',');
            }
            Serial.print("torque_ref");
        }
        Serial.println(";");
    }


    String output = "";

    drvSys_driveState data = drvSys_get_drive_state();
    drvSys_driveTargets targets = drvSys_get_targets();


    bool prev_output = false;
    if (jCtrl_state_output_config.pos) {
        float pos = data.joint_pos;

        output = output + String(pos);
    };
    if (jCtrl_state_output_config.vel) {
        float vel = data.joint_vel;

        if (prev_output) {
            output = output + ",";
        }
        output = output + String(vel);
    };
    if (jCtrl_state_output_config.acc) {
        float acc = data.joint_acc;


        if (prev_output) {
            output = output + ",";
        };
        output = output + String(acc);
    };

    if (jCtrl_state_output_config.m_torque) {
        if (prev_output) {
            output = output + ",";
        };

        float m_torque = data.motor_torque;

        output = output + String(m_torque);
    }

    if (jCtrl_state_output_config.pos_target) {
        if (prev_output) {
            output = output + ",";
        };

        float pos_target = targets.pos_target;

        output = output + pos_target;
    }

    if (jCtrl_state_output_config.vel_target) {
        if (prev_output) {
            output = output + ",";
        };

        float vel_target = targets.pos_target;

        output = output + vel_target;
    }

    if (jCtrl_state_output_config.m_torque_target) {
        if (prev_output) {
            output = output + ",";
        };

        float m_torque_target = targets.motor_torque_target;

        output = output + m_torque_target;
    }

    Serial.println();

    xSemaphoreGive(glob_Serial_mutex);

}


void _jCtrl_parse_asci_command(String asci_command) {
    int32_t index_divider = asci_command.indexOf(':');
    String keyword = asci_command.substring(0, index_divider);
    float arguments[10] = { 0 };
    bool end_of_command_reached = 0;
    int32_t n_argument = 0;
    int32_t index = index_divider;
    int32_t next_delimiter_at = asci_command.indexOf(',', index_divider + 1);
    while (!end_of_command_reached) {

        arguments[n_argument] = asci_command.substring(index + 1, next_delimiter_at).toFloat();
        index = next_delimiter_at;
        next_delimiter_at = asci_command.indexOf(',');
        if (next_delimiter_at == -1) {
            end_of_command_reached = true;
        }
        else {
            n_argument++;
        }
    }

    Serial.print(keyword);
    Serial.print(',');
    Serial.println(arguments[0]);

    if (keyword == "t") {
        float target_torque = arguments[0];

        drvSys_set_target_motor_torque(target_torque);
        return;

    }

    if (keyword == "ph") {
        float phaseshift = arguments[0];

        _drvSys_set_empiric_phase_shift(phaseshift);
        return;
    }

    if (keyword == "phoff") {
        int32_t phase_offset = arguments[0];

    }
    if (keyword == "pid_vel") {
        float P = arguments[0];
        float I = arguments[1];
        float D = arguments[2];
        bool save = bool(arguments[3]);

        drvSys_set_vel_PID_gains(P, I, D, save);


    }
    if (keyword == "pid_pos") {
        float P = arguments[0];
        float I = arguments[1];
        float D = arguments[2];
        bool save = bool(arguments[3]);

        drvSys_set_pos_PID_gains(P, I, D, save);

    }

    if (keyword == "adm_params") {
        float virt_spring = arguments[0];
        float virt_damping = arguments[1];
        float virt_inertia = arguments[2];
        bool save = bool(arguments[3]);

        drvSys_set_admittance_params(virt_spring, virt_damping, virt_inertia, save);

    }

    if (keyword == "vel") {
        float target_vel = arguments[0];

        //drvSys_set_target_velocity(target_vel);
        drvSys_set_feed_forward_velocity(target_vel);
    }

    if (keyword == "pos") {
        float target_pos = arguments[0];

        drvSys_set_target_pos(target_pos);
    }

    if (keyword == "test_sig") {
        int target_key = arguments[0]; // 0 1 2
        int target_shape = arguments[1]; // 0 1 2

        float target_max = arguments[2];
        float target_min = arguments[3];
        float period = arguments[4];

        drvSys_start_test_signal(target_key, target_shape, target_max, target_min, period);

    }

    if (keyword == "stop_test_sig") {

        drvSys_stop_test_signal();
    }

    if (keyword == "stop") {
        drvSys_stop_controllers();
    }

    if (keyword == "start") {
        drvSys_controlMode control_mode = drvSys_controlMode(arguments[0]);

        drvSys_start_motion_control(control_mode);
    }

    if (keyword == "driveInfo") {

        drvSys_controllerState ctrl_state = drvSys_get_controllerState();
    }

    if (keyword == "driveParams") {
        drvSys_parameters ctrl_parameters = drvSys_get_parameters();
    }

    if (keyword == "pid_pos_show") {
        jCtrl_show_current_pid_params(position_c);
    }
    if (keyword == "pid_vel_show") {
        jCtrl_show_current_pid_params(velocity_c);
    }
    if (keyword == "adm_show") {
        jCtrl_show_current_pid_params(admittance_c);
    }


};

void jCtrl_show_current_pid_params(controllerType c_type) {

    drvSys_parameters ctrl_parameters = drvSys_get_parameters();

    if (c_type == position_c) {


        float P = ctrl_parameters.pos_pid_gains.K_p;
        float I = ctrl_parameters.pos_pid_gains.K_i;
        float D = ctrl_parameters.pos_pid_gains.K_d;
        String output = "Position Controller PID: P =" + String(P) + ", I =" + String(I) + ", D =" + String(D);

        Serial.println(output);
    }

    if (c_type == velocity_c) {


        float P = ctrl_parameters.vel_pid_gains.K_p;
        float I = ctrl_parameters.vel_pid_gains.K_i;
        float D = ctrl_parameters.vel_pid_gains.K_d;
        String output = "Velocity Controller PID: P =" + String(P) + ", I =" + String(I) + ", D =" + String(D);

        Serial.println(output);
    }

    if (c_type == admittance_c) {

        float virtual_spring = ctrl_parameters.admittance_gains.virtual_spring;
        float virtual_damping = ctrl_parameters.admittance_gains.virtual_damping;
        float virtual_inertia = ctrl_parameters.admittance_gains.virtual_inertia;

        String output = "Admittance Controller: Spring Const. : " + String(virtual_spring) + ", Damping: " +
            String(virtual_damping) + ", Inerta: " + String(virtual_inertia);

        Serial.println(output);
    }


}