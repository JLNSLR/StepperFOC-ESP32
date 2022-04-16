#ifndef DRIVE_KALMAN_FILTER_H
#define DRIVE_KALMAN_FILTER_H

#include <BasicLinearAlgebra.h>
#include <Arduino.h>


#define DEG2RAD 0.01745329251994329576923690768489
#define RAD2DEG 57.295779513082320876798154814105


class DriveKalmanFilter {
public:
    DriveKalmanFilter();
    DriveKalmanFilter(float delta_t, float transmission);

    void prediction_step();
    void correction_step();

    void add_motor_encoder_measurement(float angle_deg, float angle_vel_deg, float angle_acc_deg);
    void add_joint_encoder_measurement(float angle_deg);
    void add_torque_measurement();

    void set_drive_input(float motor_torque);

    void get_total_friction_estimate();

    float get_load_torque();
    float get_delta_angle();
    float get_angle_deg();
    float get_vel_deg();
    float get_acc_deg();

    void scale_system_noise_matrix(float scale_factor);


    enum kalman_state_vector {
        kalman_angle, kalman_vel, kalman_acc, kalman_load_torque, kalman_motor_angle, kalman_inertia,
        kalman_delta_angle, kalman_visc_fric_coef, kalman_coloumb_fric_coef, kalman_stiction_coef
    };


    /* Kalman Matrices and Vectors */

    BLA::Matrix<10> x_pred;
    BLA::Matrix<10> x_corrected;

    BLA::Matrix<10> x_current;


    BLA::Matrix<10> b_vector;
    BLA::Matrix<10, 10> system_matrix_A;
    BLA::Matrix<10, 10> kalman_gain;
    BLA::Matrix<10, 10> error_covariance_matrix;


    BLA::Matrix<10, 10> system_noise_matrix;
    BLA::Matrix<10, 10> measurement_noise_matrix;

    BLA::Matrix<10> x_initial;

    float input_torque = 0.0;


    /* Drive Parameters */
    float delta_t = 0.0005;
    float transmission = 50;


    float pos_enc_measurement_noise = 0.01 * DEG2RAD; // in deg
    float vel_enc_measurement_moise = 0.5 * DEG2RAD; // in deg/s
    float acc_enc_measurement_noise = 5.0 * DEG2RAD; // deg/s^2

    float joint_torque_measurement_noise = 0.5; // N_m

    float joint_encoder_pos_measurement_noise = 0.1; // deg

    // System noise parameters 
    float sys_noise_pos = 0.001 * DEG2RAD; // deg
    float sys_noise_vel = 0.01 * DEG2RAD; // deg/s
    float sys_noise_acc = 5.0 * DEG2RAD; //deg/s^2;
    float sys_noise_load_torque = 1.0; // N_m;
    float sys_noise_motor_angle = 0.05 * DEG2RAD; // deg
    float sys_noise_delta_angle = 0.05 * DEG2RAD;
    float sys_noise_inertia = 0.1; // kgm^2;
    float sys_visc_friction = 0.05; // Nm/(deg/s)
    float sys_coloumb_friction = 0.05; // Nm
    float sys_stiction = 0.05; //Nm


private:
    BLA::Matrix<10, 10> output_matrix_motor_enc;
    BLA::Matrix<10, 10> output_matrix_joint_enc;
    BLA::Matrix<10, 10> output_matrix_torque_sensor;

    float getJ();

    float get_coloumb_friction(float angle_vel, float coloumb_friction_coefficient);
    float get_stiction(float angle_vel, float stiction_coefficient);

    const float stiction_velocity_limit = 0.1 * DEG2RAD; // deg/s

    BLA::Matrix(10, 10) identity_mat;



};

#endif //DRIVE_KALMAN_FILTER_H