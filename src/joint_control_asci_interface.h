
#ifndef JOINT_CONTROL_ASCI_INTERFACE
#define JOINT_CONTROL_ASCI_INTERFACE
#include <drive_system.h>


const int jCtrl_asci_interface_period_ms = 10;

enum targetType { position_target, velocity_target, torque_target, refTorque_target };

enum controllerType { position_c, velocity_c, admittance_c };

extern bool jCtrl_active;

void jCtrl_asci_start_interface();

void _jCtrl_asci_task(void* params);

void jCtrl_asci_debug_output_start(bool pos = false, bool vel = false, bool acc = false, bool m_torque = false, bool target_pos = false, bool target_vel = false);

void _jCtrl_asci_handle_debug_output();

void jCtrl_handle_target_command(targetType type, float target_val);

void jCtrl_handle_pid_command(controllerType c_type, float P, float I, float D);

void jCtrl_show_current_pid_params(controllerType c_type);

void _jCtrl_parse_asci_command(String asci_command);












#endif //JOINT_CONTROL_ASCI_INTERFACE
