#include "Copter.h"
#include <AP_Motors/AP_MotorsMatrix.h>
#include <GCS_MAVLink/GCS.h>

/*
 * Init and run calls for stabilize flight mode
 */

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
    Vector3f velocity;
    bool a = ahrs_.get_velocity_NED(velocity);
    // gcs().send_text(MAV_SEVERITY_DEBUG, "u %5.3f", (double)velocity[0]);
    Vector3f rates = ahrs_.get_gyro_latest();
    // gcs().send_text(MAV_SEVERITY_DEBUG, "q %5.3f", (double)rates[0]);

    // End custom code


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
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->reset_rate_controller_I_terms();
        break;
    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->set_yaw_target_to_current_heading();
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


    // Begin custom code

    // Assign commands to the custom controller
    ExtU rtU_;
    ExtY rtY_;
    rtU_.ch18[0] = yaw_out;
    rtU_.ch18[1] = - throttle_control;
    rtU_.ch18[2] = roll_out;
    rtU_.ch18[3] = pitch_out;
    rtU_.ch18[4] = 0.0f;
    rtU_.ch18[5] = 0.0f;
    rtU_.ch18[6] = 0.0f;
    rtU_.ch18[7] = 0.0f;
    //gcs().send_text(MAV_SEVERITY_DEBUG, "ch1 %5.3f", (double)rtU_.ch18[0]);

    // Assign measures to the custom controller
    rtU_.OutputMeasure[0] = rates[0];
    rtU_.OutputMeasure[1] = rates[1];
    rtU_.OutputMeasure[2] = rates[2];
    float roll_angle = attitude_vehicle_quat.get_euler_roll();
    float pitch_angle = attitude_vehicle_quat.get_euler_pitch();
    float yaw_angle = attitude_vehicle_quat.get_euler_yaw();
    rtU_.OutputMeasure[3] = roll_angle;
    rtU_.OutputMeasure[4] = pitch_angle;
    rtU_.OutputMeasure[5] = yaw_angle;
    rtU_.OutputMeasure[6] = velocity[0];
    rtU_.OutputMeasure[7] = velocity[1];
    rtU_.OutputMeasure[8] = velocity[2];

    // Assign controller inputs to the controller object
    controller.rtU = rtU_;

    // Perform a controller executation
    controller.step();

    // Get controller output
    rtY_ = controller.rtY;

    // real32_T u1 = rtY_.u[0];
    // gcs().send_text(MAV_SEVERITY_DEBUG, "u1 %5.3f", u1);


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
        motors->set_custom_input( 0, rtY_.u[1] );
        motors->set_custom_input( 1, rtY_.u[3] );
        motors->set_custom_input( 2, rtY_.u[0] );
        motors->set_custom_input( 3, rtY_.u[2] );
        break;
    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        motors->set_custom_input( 0, rtY_.u[1] );
        motors->set_custom_input( 1, rtY_.u[3] );
        motors->set_custom_input( 2, rtY_.u[0] );
        motors->set_custom_input( 3, rtY_.u[2] );
        // do nothing
        break;
    }

    gcs().send_text(MAV_SEVERITY_DEBUG, "test   ");

}
