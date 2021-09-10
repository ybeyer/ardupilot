#include "Copter.h"
#include <AP_Motors/AP_MotorsMatrix.h>
#include <GCS_MAVLink/GCS.h>

// Function for hardcoding changes to MATLABs cntrl struct.
// Values can be accessed in the same fashion as in MATLAB, e.g.:
//     cntrl.sample_time = 42;
void ModeCustom::override_cntrl_params()
{

}

/*
 * Init and run calls for stabilize flight mode
 */
bool ModeCustom::init(bool ignore_checks)
{
    override_cntrl_params();
    // initialize yaw to measured value
    const AP_AHRS_View &ahrs_ = attitude_control->get_ahrs();
    Quaternion attitude_vehicle_quat;
    ahrs_.get_quat_body_to_ned(attitude_vehicle_quat);
    yawInit = attitude_vehicle_quat.get_euler_yaw();

    // initialize position to measured value
    Vector3f position_NED;
    bool b = ahrs_.get_relative_position_NED_home(position_NED);
    sInit[0] = position_NED[0];
    sInit[1] = position_NED[1];
    sInit[2] = position_NED[2];

    // tell the controller to use the initial conditions on the first time step
    custom_controller.initialize();

    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeCustom::run()
{
    // apply simple mode transform to pilot inputs
    update_simple_mode();


    // Begin custom code

    // Get stick inputs, -1 ... 1
    int16_t tr_max = 4500;
    // fetch roll and pitch inputs
    float roll_out_high = channel_roll->get_control_in();
    float roll_out = roll_out_high / tr_max;
    float pitch_out_high = channel_pitch->get_control_in();
    float pitch_out = pitch_out_high / tr_max;
    float throttle_control_high = channel_throttle->get_control_in();
    float throttle_control = throttle_control_high / 1000 * 2 - 1;
    // get pilot's desired yaw rate
    float yaw_out_high = channel_yaw->get_control_in();
    float yaw_out = yaw_out_high / tr_max;

    // Get measured values
    // Retrieve quaternion vehicle attitude
    const AP_AHRS_View& ahrs_ = attitude_control->get_ahrs();
    Quaternion attitude_vehicle_quat;
    ahrs_.get_quat_body_to_ned(attitude_vehicle_quat);
    // Get velocity relative to the ground in NED
    //bool check = ahrs_.have_inertial_nav(void);
    Vector3f velocity_NED;
    bool a = ahrs_.get_velocity_NED(velocity_NED);
    // gcs().send_text(MAV_SEVERITY_DEBUG, "u %5.3f", (double)velocity[0]);

    Vector3f angular_velocity_Kb = ahrs_.get_gyro_latest();

    // gcs().send_text(MAV_SEVERITY_DEBUG, "q %5.3f", (double)rates[0]);
    float roll_angle = attitude_vehicle_quat.get_euler_roll();
    float pitch_angle = attitude_vehicle_quat.get_euler_pitch();
    float yaw_angle = attitude_vehicle_quat.get_euler_yaw();
    // Get position relative to the ground in NED
    Vector3f position_NED;
    bool b = ahrs_.get_relative_position_NED_home(position_NED);
    // end custom code

    // convert pilot input to lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero) {
        // Attempting to Land
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        break;
    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        break;
    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;
    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

    // call attitude controller
    //attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // output pilot's throttle
    //attitude_control->set_throttle_out(get_pilot_desired_throttle(),
    //                                   true,
    //                                   g.throttle_filt);


    // BEGIN NEW CODE

    ExtU rtU_;
    ExtY rtY_;

    rtU_.cmd.roll = roll_out;
    rtU_.cmd.pitch = pitch_out;
    rtU_.cmd.yaw = yaw_out;
    rtU_.cmd.thr = -throttle_control;
    rtU_.cmd.s_Kg_init[0] = sInit[0];
    rtU_.cmd.s_Kg_init[1] = sInit[1];
    rtU_.cmd.s_Kg_init[2] = sInit[2];
    rtU_.cmd.yaw_init = yawInit;
    for (int i=0;i<16;i++) {
        rtU_.cmd.RC_pwm[i] = g2.rc_channels.channel(i)->get_radio_in();
    }
    real32_T debug_var = throttle_control;
    // gcs().send_text(MAV_SEVERITY_DEBUG, "debug var %5.3f", (float)debug_var);

    rtU_.measure.omega_Kb[0] = angular_velocity_Kb[0];
    rtU_.measure.omega_Kb[1] = angular_velocity_Kb[1];
    rtU_.measure.omega_Kb[2] = angular_velocity_Kb[2];
    rtU_.measure.q_bg[0] = attitude_vehicle_quat.q1;
    rtU_.measure.q_bg[1] = attitude_vehicle_quat.q2;
    rtU_.measure.q_bg[2] = attitude_vehicle_quat.q3;
    rtU_.measure.q_bg[3] = attitude_vehicle_quat.q4;
    rtU_.measure.EulerAngles[0] = roll_angle;
    rtU_.measure.EulerAngles[1] = pitch_angle;
    rtU_.measure.EulerAngles[2] = yaw_angle;
    rtU_.measure.a_Kg[0] = ahrs_.get_accel_ef_blended().x;
    rtU_.measure.a_Kg[1] = ahrs_.get_accel_ef_blended().y;
    rtU_.measure.a_Kg[2] = ahrs_.get_accel_ef_blended().z;
    rtU_.measure.V_Kg[0] = velocity_NED[0];
    rtU_.measure.V_Kg[1] = velocity_NED[1];
    rtU_.measure.V_Kg[2] = velocity_NED[2];
    rtU_.measure.s_Kg[0] = position_NED[0];
    rtU_.measure.s_Kg[1] = position_NED[1];
    rtU_.measure.s_Kg[2] = position_NED[2];
    rtU_.measure.lla[0] = copter.current_loc.lat;
    rtU_.measure.lla[1] = copter.current_loc.lng;
    rtU_.measure.lla[2] = copter.current_loc.alt;

    custom_controller.rtU = rtU_;
    custom_controller.step();
    rtY_ = custom_controller.rtY;

    // real32_T u1 = rtY_.u[0];
    // gcs().send_text(MAV_SEVERITY_DEBUG, "u1 %5.3f", u1);

    // log data
    AP::logger().Write(
        "ML", "TimeUS,v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12,v13,v14,v15",
        "Qfffffffffffffff",
        AP_HAL::micros64(),
        (double)custom_controller.rtY.logs[0],
        (double)custom_controller.rtY.logs[1],
        (double)custom_controller.rtY.logs[2],
        (double)custom_controller.rtY.logs[3],
        (double)custom_controller.rtY.logs[4],
        (double)custom_controller.rtY.logs[5],
        (double)custom_controller.rtY.logs[6],
        (double)custom_controller.rtY.logs[7],
        (double)custom_controller.rtY.logs[8],
        (double)custom_controller.rtY.logs[9],
        (double)custom_controller.rtY.logs[10],
        (double)custom_controller.rtY.logs[11],
        (double)custom_controller.rtY.logs[12],
        (double)custom_controller.rtY.logs[13],
        (double)custom_controller.rtY.logs[14]);

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        //attitude_control->set_yaw_target_to_current_heading();
        //attitude_control->reset_rate_controller_I_terms();
        break;
    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        // sends output to motors when armed but not flying
        //attitude_control->set_yaw_target_to_current_heading();
        //attitude_control->reset_rate_controller_I_terms();
        break;
    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        motors->set_custom_input( 0, rtY_.u[0] );
        motors->set_custom_input( 1, rtY_.u[1] );
        motors->set_custom_input( 2, rtY_.u[2] );
        motors->set_custom_input( 3, rtY_.u[3] );
        break;
    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        motors->set_custom_input( 0, rtY_.u[0] );
        motors->set_custom_input( 1, rtY_.u[1] );
        motors->set_custom_input( 2, rtY_.u[2] );
        motors->set_custom_input( 3, rtY_.u[3] );
        // do nothing
        break;
    }
}
