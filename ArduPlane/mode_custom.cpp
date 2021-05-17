#include "mode.h"
#include "Plane.h"
#include <AP_Common/MatlabController.h>

bool ModeCustom::_enter()
{
    custom_controller.initialize(); 
    return true;
}


void ModeCustom::update()
{

    // get pilot inputs
    float tr_max_inv = 1.0 / 4500;
    float roll_out_high = plane.channel_roll->get_control_in();
    float roll_out = roll_out_high * tr_max_inv;
    float pitch_out_high = plane.channel_pitch->get_control_in();
    float pitch_out = pitch_out_high * tr_max_inv;
    float yaw_out_high = plane.channel_rudder->get_control_in();
    float yaw_out = yaw_out_high * tr_max_inv;
    float throttle_control_high = plane.channel_throttle->get_control_in();
    float throttle_control = 0.01 * throttle_control_high;

    // get rangefinder distance in cm (if sensor with orientation is avail.)

    float rangefinder_dist[6];
    rangefinder_dist[0] = plane.rangefinder.distance_cm_orient((Rotation)1);
    rangefinder_dist[1] = plane.rangefinder.distance_cm_orient((Rotation)2);
    rangefinder_dist[2] = plane.rangefinder.distance_cm_orient((Rotation)3);
    rangefinder_dist[3] = plane.rangefinder.distance_cm_orient((Rotation)4);
    rangefinder_dist[4] = plane.rangefinder.distance_cm_orient((Rotation)5);
    rangefinder_dist[5] = plane.rangefinder.distance_cm_orient((Rotation)6);

    int   RNGFND_num_sensors = plane.rangefinder.num_sensors();

      // get measured inputs
    Vector3f angular_velocity_Kb = plane.ahrs.get_gyro();

    Quaternion attitude_vehicle_quat;
    // bool a = plane.ahrs.get_quaternion(attitude_vehicle_quat);
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

    rtU_.cmd.roll  = roll_out;
    rtU_.cmd.pitch = pitch_out;
    rtU_.cmd.yaw   = yaw_out;
    rtU_.cmd.thr   = throttle_control;

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
    rtU_.measure.rangefinder[0] = rangefinder_dist[0];
    rtU_.measure.rangefinder[1] = rangefinder_dist[1];
    rtU_.measure.rangefinder[2] = rangefinder_dist[2];
    rtU_.measure.rangefinder[3] = rangefinder_dist[3];
    rtU_.measure.rangefinder[4] = rangefinder_dist[4];
    rtU_.measure.rangefinder[5] = rangefinder_dist[5];



    // get controller outputs struct
    custom_controller.rtU = rtU_;
    custom_controller.step(); //run a step in controller.
    ExtY rtY_ = custom_controller.rtY;

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

    // send controller outputs to channels and set PWMs
    for (uint8_t i=0; i<8; i++) {
        SRV_Channel::Aux_servo_function_t function_i = (SRV_Channel::Aux_servo_function_t)rtY_.function_channels[i];

        if(function_i == SRV_Channel::Aux_servo_function_t::k_none){
            continue;  
        }
        if(function_i == SRV_Channel::Aux_servo_function_t::k_throttle) {
            if (!hal.util->get_soft_armed()) {
                if (plane.arming.arming_required() == AP_Arming::Required::YES_ZERO_PWM) {
                    SRV_Channels::set_output_limit(function_i, SRV_Channel::Limit::ZERO_PWM);
                } else {
                    SRV_Channels::set_output_norm(function_i, 0);
                }
            }
            else{
                SRV_Channels::set_output_norm(function_i, rtY_.channels[i]);
            }
        }
        else
        {
             SRV_Channels::set_output_norm(function_i, rtY_.channels[i]);
        }
    }
    // plot rangefinder distance in cm for debugging

    static uint16_t counter;
    static uint32_t last_t, last_print;
    uint32_t now = AP_HAL::micros();

    if (last_t == 0) {
        last_t = now;
        return;
    }
    last_t = now;

    counter++;

    if (now - last_print >= 100000 /* 100ms : 10hz */) {

       GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "%d: distance_cm %f %f %f %f %f %f \n",
        RNGFND_num_sensors,
        rangefinder_dist[0], rangefinder_dist[1], rangefinder_dist[2],
        rangefinder_dist[3], rangefinder_dist[4], rangefinder_dist[5]);
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Servo PWM: %f %f %f %f %f %f %f %f \n",
         rtY_.channels[0], rtY_.channels[1], rtY_.channels[2], rtY_.channels[3],
         rtY_.channels[4], rtY_.channels[5], rtY_.channels[6], rtY_.channels[7]);

        last_print = now;
        counter = 0;
    }


}
