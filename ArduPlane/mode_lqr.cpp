#include "mode.h"
#include "Plane.h"
#include "quadplane.h"
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_Motors/AP_Motors.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Common/MatlabController.h>

bool ModeCustom::_enter() //copy of mode_manual, but with vtol_mode
{
    
    
    // plane.throttle_allows_nudging = false;
    // plane.auto_throttle_mode = false; //dont call speed/height-controller
    // plane.auto_navigation_mode = false;
    // plane.auto_state.vtol_mode = true; //this might be problematic since the line above says that we're not in auto-mode

    return true;
}




void ModeCustom::update() 
{
    
    int16_t tr_max = 4500;

    float roll_out_high = plane.channel_roll->get_control_in();
    float roll_out = roll_out_high / tr_max;
    float pitch_out_high = plane.channel_pitch->get_control_in();
    float pitch_out = pitch_out_high / tr_max;
    float yaw_out_high = plane.channel_rudder->get_control_in();
    float yaw_out = yaw_out_high / tr_max;
    float throttle_control_high = plane.channel_throttle->get_control_in();
    float throttle_control = throttle_control_high / 100 * 2 - 1;



    Vector3f angular_velocity_Kb = plane.ahrs.get_gyro();

    Quaternion attitude_vehicle_quat;
    bool a = plane.ahrs.get_quaternion(attitude_vehicle_quat);
    // gcs().send_text(MAV_SEVERITY_DEBUG, "a %d", (int8_t)a);
    // gcs().send_text(MAV_SEVERITY_DEBUG, "test %5.3f %5.3f %5.3f %5.3f", (double)attitude_vehicle_quat[0], (double)attitude_vehicle_quat[1], attitude_vehicle_quat[2], attitude_vehicle_quat[3]);

    float roll = plane.ahrs.get_roll();
    float pitch = plane.ahrs.get_pitch();
    float yaw = plane.ahrs.get_yaw();

    Vector3f acc_NED = plane.ahrs.get_accel_ef_blended();

    Vector3f velocity_NED;
    plane.ahrs.get_velocity_NED(velocity_NED);

    Vector3f position_NED;
    plane.ahrs.get_relative_position_NED_home(position_NED);



    ExtU rtU_;
    ExtY rtY_;


    rtU_.cmd.roll = roll_out;
    rtU_.cmd.pitch = pitch_out;
    rtU_.cmd.yaw = yaw_out;
    rtU_.cmd.thr = throttle_control;

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


    custom_controller.rtU = rtU_;
    custom_controller.step(); //run a step in controller. 
    rtY_ = custom_controller.rtY;

    SRV_Channels::set_output_pwm(SRV_Channel::k_elevon_left, (rtY_.u[0])*1000+1000); //set output values
    SRV_Channels::set_output_pwm(SRV_Channel::k_elevon_right, (rtY_.u[1])*1000+1000);
    SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, rtY_.u[2]*1000+1000);
    SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, rtY_.u[3]*1000+1000);

}
