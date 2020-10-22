#include "mode.h"
#include "Plane.h"
#include <AP_Common/MatlabController.h>

bool ModeCustom::_enter()
{
    return true;
}


void ModeCustom::update() 
{
    
    // get pilot inputs
    int16_t tr_max = 4500;
    float roll_out_high = plane.channel_roll->get_control_in();
    float roll_out = roll_out_high / tr_max;
    float pitch_out_high = plane.channel_pitch->get_control_in();
    float pitch_out = pitch_out_high / tr_max;
    float yaw_out_high = plane.channel_rudder->get_control_in();
    float yaw_out = yaw_out_high / tr_max;
    float throttle_control_high = plane.channel_throttle->get_control_in();
    float throttle_control = throttle_control_high / 100 * 2 - 1;


    // get measured inputs
    Vector3f angular_velocity_Kb = plane.ahrs.get_gyro();

    Quaternion attitude_vehicle_quat;
    bool a = plane.ahrs.get_quaternion(attitude_vehicle_quat);
    // gcs().send_text(MAV_SEVERITY_DEBUG, "a %d", (int8_t)a);
    // gcs().send_text(MAV_SEVERITY_DEBUG, "test %5.3f %5.3f %5.3f %5.3f", (double)attitude_vehicle_quat[0], (double)attitude_vehicle_quat[1], attitude_vehicle_quat[2], attitude_vehicle_quat[3]);

    AP::logger().Write("QUAT", "TimeUS,q0,q1,q2,q3", "Qffff",
                                        AP_HAL::micros64(),
                                        (double)attitude_vehicle_quat[0],
                                        (double)attitude_vehicle_quat[1],
                                        (double)attitude_vehicle_quat[2],
                                        (double)attitude_vehicle_quat[3]);

    Vector3f acc_NED = plane.ahrs.get_accel_ef_blended();

    Vector3f velocity_NED;
    plane.ahrs.get_velocity_NED(velocity_NED);

    Vector3f position_NED;
    plane.ahrs.get_relative_position_NED_home(position_NED);


    // assign commanded and measured values to controller inputs struct
    ExtU rtU_;

    rtU_.cmd.roll = roll_out;
    rtU_.cmd.pitch = pitch_out;
    rtU_.cmd.yaw = yaw_out;
    rtU_.cmd.thr = -throttle_control;

    rtU_.measure.omega_Kb[0] = angular_velocity_Kb[0];
    rtU_.measure.omega_Kb[1] = angular_velocity_Kb[1];
    rtU_.measure.omega_Kb[2] = angular_velocity_Kb[2];
    rtU_.measure.q_bg[0] = attitude_vehicle_quat[0];
    rtU_.measure.q_bg[1] = attitude_vehicle_quat[1];
    rtU_.measure.q_bg[2] = attitude_vehicle_quat[2];
    rtU_.measure.q_bg[3] = attitude_vehicle_quat[3];
    rtU_.measure.EulerAngles[0] = plane.ahrs.roll;
    rtU_.measure.EulerAngles[1] = plane.ahrs.pitch;
    rtU_.measure.EulerAngles[2] = plane.ahrs.yaw;
    rtU_.measure.a_Kg[0] = acc_NED.x;
    rtU_.measure.a_Kg[1] = acc_NED.y;
    rtU_.measure.a_Kg[2] = acc_NED.z;
    rtU_.measure.V_Kg[0] = velocity_NED[0];
    rtU_.measure.V_Kg[1] = velocity_NED[1];
    rtU_.measure.V_Kg[2] = velocity_NED[2];
    rtU_.measure.s_Kg[0] = position_NED[0];
    rtU_.measure.s_Kg[1] = position_NED[1];
    rtU_.measure.s_Kg[2] = position_NED[2];
    rtU_.measure.lla[0] = plane.current_loc.lat;
    rtU_.measure.lla[1] = plane.current_loc.lng;
    rtU_.measure.lla[2] = plane.current_loc.alt;


    // get controller outputs struct
    custom_controller.rtU = rtU_;
    custom_controller.step(); //run a step in controller. 
    ExtY rtY_ = custom_controller.rtY;


    // send controller outputs to channels and set PWMs
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        SRV_Channel::Aux_servo_function_t function_i = SRV_Channels::channel_function(i);
        SRV_Channel *c = SRV_Channels::srv_channel(i);
        bool is_servo = c->is_servo();
        float u_norm = rtY_.u[i];
        if (is_servo) {
            u_norm = rtY_.u[i]*2-1;
        }
        if (c->get_reversed()) {
            u_norm = -u_norm;
        }
        // similar to servos.cpp, l. 714
        if (!hal.util->get_soft_armed() && !c->is_servo()) {
            if (plane.arming.arming_required() == AP_Arming::Required::YES_ZERO_PWM) {
                SRV_Channels::set_output_limit(function_i, SRV_Channel::Limit::ZERO_PWM);
            } else {
                SRV_Channels::set_output_norm(function_i, 0);
            }
        } else {
            SRV_Channels::set_output_norm(function_i, u_norm);
        }
    }
    

}
