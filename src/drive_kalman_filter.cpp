#include <drive_kalman_filter.h>


DriveKalmanFilter::DriveKalmanFilter(float delta_t, float transmission) {

    x_initial.Fill(0); // fill initial state vector with zeros;

    x_initial(kalman_inertia) = 1.0; // initial inertia

    x_pred.Fill(0);

    x_corrected.Fill(0);

    this->delta_t = delta_t;
    // Setup initial system matrix;
    system_matrix_A.Fill(0);
    // angle
    system_matrix_A(0, 0) = 1.0;
    systen_matrix_A(0, 1) = delta_t;
    //vel
    systen_matrix_A(1, 1) = 1.0;
    system_matrix_A(1, 2) = delta_t;
    //acc dynamics
    system_matrix_A(2, 1) = -x_initial(kalman_visc_fric_coef) / x_initial(kalman_inertia);
    system_matrix_A(2, kalman_coloumb_fric_coef) = -get_coloumb_friction(x_initial(kalman_vel), x_initial(kalman_coloumb_fric_coef)) / x_initial(kalman_inertia);
    system_matrix_A(2, kalman_stiction_coef) = -get_stiction(x_initial(kalman_vel), x_initial(kalman_stiction_coef)) / x_initial(kalman_inertia);

    //inertia dynamics
    system_matrix_A(kalman_inertia, kalman_inertia) = 1.0;
    //delta_angle
    system_matrix_A(kalman_delta_angle, kalman_delta_angle) = 1.0;
    //friction coef
    system_matrix_A(kalman_visc_fric_coef, kalman_visc_fric_coef) = 1.0;
    system_matrix_A(kalman_coloumb_fric_coef, kalman_coloumb_fric_coef) = 1.0;
    system_matrix_A(kalman_stiction_coef, kalman_stiction_coef) = 1.0;


    // setup b_vector
    b_vector.Fill(0);
    b_vector(kalman_acc) = transmission / x_initial(kalman_inertia);


    /* Setup System Noise Matrix */
    system_noise_matrix.Fill(0);
    system_noise_matrix(0, 0) = sys_noise_pos;
    system_noise_matrix(1, 1) = sys_noise_vel;
    system_noise_matrix(2, 2) = sys_noise_acc;
    system_noise_matrix(3, 3) = sys_noise_load_torque;
    system_noise_matrix(4, 4) = sys_noise_motor_angle;
    system_noise_matrix(5, 5) = sys_noise_inertia;
    system_noise_matrix(6, 6) = sys_noise_delta_angle;
    system_noise_matrix(7, 7) = sys_visc_friction;
    system_noise_matrix(8, 8) = sys_coloumb_friction;
    system_noise_matrix(9, 9) = sys_stiction;


    /* Setup Measurement Noise matrices */

    measurement_noise_matrix.Fill(0);
    measurement_noise_matrix(0, 0) = pos_enc_measurement_noise;
    measurement_noise_matrix(1, 1) = vel_enc_measurement_moise;
    measurement_noise_matrix(2, 2) = acc_enc_measurement_noise;
    measurement_noise_matrix(3, 3) = joint_torque_measurement_noise;

    /* Setup output matrices */

    output_matrix_motor_enc.Fill(0.0);
    output_matrix_motor_enc(0, 0) = 1.0;
    output_matrix_motor_enc(1, 1) = 1.0;
    output_matrix_motor_enc(2, 2) = 1.0;
    output_matrix_motor_enc(4, 4) = 1.0;

    output_matrix_joint_enc.Fill(0.0);
    output_matrix_joint_enc(0, 0) = 1.0;
    output_matrix_joint_enc(1, 1) = 1.0;
    output_matrix_joint_enc(2, 2) = 1.0;
    output_matrix_joint_enc(kalman_delta_angle, kalman_delta_angle) = 1.0;


    output_matrix_torque_sensor.Fill(0);
    output_matrix_torque_sensor(kalman_load_torque, kalman_load_torque) = 1.0;

    x_corrected = x_initial;
    x_current = x_initial;

    /* setup identify mat */
    identity_mat.Fill(0);
    for (int i = 0; i < 10; i++) {
        identity_mat(i, i) = 1.0;
    }


};

void DriveKalmanFilter::prediction_step() {
    //setup System matrix;

    //predict next state_vector

    x_pred = x_current;
    x_pred(kalman_angle) = 0;
    x_pred(kalman_vel) = 0;
    x_pred(kalman_acc) = 0;
    x_pred(kalman_motor_angle) = 0;

    x_pred(kalman_angle) = x_current(kalman_angle) + delta_t * x_current(kalman_vel);
    x_pred(kalman_vel) = x_current(kalman_vel) + delta_t * x_current(kalman_acc);

    x_pred(kalman_acc) = (1 / x_current(kalman_inertia)) * (input_torque * transmission - get_total_friction_estimate() - get_load_torque())
        x_pred(kalman_load_torque) = x_current(kalman_load_torque);
    x_pred(kalman_motor_angle) = x_current(kalman_angle) / N - x_current(kalman_delta_angle);


    // Update Error Covariance Matrix

    //Update nonlinear elements of System matrix
    system_matrix_A(2, 1) = -x_current(kalman_visc_fric_coef) / x_current(kalman_inertia);
    system_matrix_A(2, kalman_coloumb_fric_coef) = -get_coloumb_friction(x_current(kalman_vel), x_current(kalman_coloumb_fric_coef)) / x_current(kalman_inertia);
    system_matrix_A(2, kalman_stiction_coef) = -get_stiction(x_current(kalman_vel), x_current(kalman_stiction_coef)) / x_current(kalman_inertia);

    error_covariance_matrix = system_matrix_A * error_covariance_matrix * ~system_matrix_A + system_noise_matrix;

    x_current = x_pred;


};

void DriveKalmanFilter::correction_step(BLA::Matrix<10> y_meas, BLA::Matrix<10, 10> C_output_matrix, BLA::Matrix<10, 10> measurement_noise_matrix) {

    //Calculate Kalman Gain
    kalman_gain = error_covariance_matrix * ~C_output_matrix * BLA::Invert(C_output_matrix * error_covariance_matrix * ~C_output + measurement_noise_matrix);
    //Calculate corrected state vector
    x_corrected = x_current + kalman_gain * (y_meas - C_output_matrix * x_current);
    //update error covariance matrix
    error_covariance_matrix = (identity_mat + kalman_gain * C_output_matrix) * error_covariance_matrix;
};

void DriveKalmanFilter::add_joint_encoder_measurement(float angle_deg, float angle_vel_deg, float angle_acc_deg, float delta_angle = 0.0) {
    BLA::Matrix<10> y_meas;
    y_meas.Fill(0);
    y_meas(kalman_angle) = angle_deg * DEG2RAD;
    y_meas(kalman_vel) = angle_vel_deg * DEG2RAD;
    y_meas(kalman_acc) = angle_acc_deg * DEG2RAD;
    y_meas(kalman_delta_angle) = delta_angle * DEG2RAD;

    correction_step(y_meas, output_matrix_joint_enc, measurement_noise_matrix * 1.5);

    x_current = x_corrected;

};

void DriveKalmanFilter::add_motor_encoder_measurement(float angle_deg, float angle_vel_deg, float angle_acc_deg, float motor_angle_deg) {
    // Calculate Kalman Gain

    BLA::Matrix<10> y_meas;
    y_meas.Fill(0);
    y_meas(kalman_angle) = angle_deg * DEG2RAD;
    y_meas(kalman_vel) = angle_vel_deg * DEG2RAD;
    y_meas(kalman_acc) = angle_acc_deg * DEG2RAD;
    y_meas(kalman_motor_angle) = motor_angle_deg * DEG2RAD;

    correction_step(y_meas, output_matrix_motor_enc, measurement_noise_matrix);

    x_current = x_corrected;

};

void DriveKalmanFilter::add_torque_measurement(float torque) {
    BLA::Matrix<10> y_meas;
    y_meas.Fill(0);
    y_meas(kalman_load_torque) = torque;

    correction_step(y_meas, output_matrix_torque_sensor, measurement_noise_matrix);

    x_current = x_corrected;
}

float DriveKalmanFilter::get_coloumb_friction(float angle_vel, float coloumb_friction_coefficient) {

    if (angle_vel > 0.0) {
        return coloumb_friction_coefficient;
    }
    else if (angle_vel < 0.0) {
        return coloumb_friction_coefficient;
    }
    else {
        return 0.0;
    }
};

float DriveKalmanFilter::get_stiction(float angle_vel, float stiction_coefficient) {

    if (angle_vel > 0.0 && angle_vel < stiction_velocity_limit) {
        return stiction_coefficient;
    }
    else if (angle_vel < 0.0 && angle_vel > stiction_velocity_limit * (-1)) {
        return stiction_coefficient * (-1.0);
    }
    return 0.0;
};

float DriveKalmanFilter::get_total_friction_estimate() {
    float coloumb_fric = get_coloumb_friction(x_current(kalman_vel), x_current(kalman_coloumb_fric_coef));
    float stiction = get_stiction(x_current(kalman_vel), x_current(kalman_stiction_coef));
    float torque_friction = x_current(kalman_vel) * x_current(kalman_visc_fric_coef) + coloumb_fric + stiction;

    return torque_friction;
};

void DriveKalmanFilter::scale_system_noise_matrix(float scale_factor) {
    system_noise_matrix = scale_factor * system_noise_matrix;
}


/* Getter Functions */

float DriveKalmanFilter::getJ() {
    return x_current(kalman_inertia);
};

float DriveKalmanFilter::get_load_torque() {
    return x_current(kalman_load_torque);
};

float DriveKalmanFilter::get_delta_angle() {
    return x_current(kalman_delta_angle) * RAD2DEG;
};

float DriveKalmanFilter::get_angle_deg() {
    return x_current(kalman_angle) * RAD2DEG;
};

float DriveKalmanFilter::get_vel_deg() {
    return x_current(kalman_vel) * RAD2DEG;
};

float DriveKalmanFilter::get_acc_deg() {
    return x_current(kalman_acc) * RAD2DEG;
}



