#include "mode.h"
#include "Plane.h"
#include <AP_Common/MatlabController.h>
#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>

// Matek H743-MINI: 1 = TX7, 2 = TX1, 3 = TX2, 6 = TX4, 7 = TX6
//#define fc_uart hal.serial(2)
AP_HAL::UARTDriver *fc_uart = hal.serial(2);

uint16_t missed_frames = 0;

bool ModeCustom::_enter()
{
    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Initialize Custom Controller ...");

    fc_uart->begin(6000000, sizeof(ExtU), sizeof(ExtY));

    custom_controller.initialize();

    missed_frames = 0;

    return true;
}

void ModeCustom::update()
{
    // assign commanded and measured values to controller inputs struct
    ExtU *rtU_ = &(custom_controller.rtU);

    // entry time of the loop for precise measurement of the cycle time in microseconds
    uint32_t time_beg = AP_HAL::micros();

    // get pilot inputs and scale them between 0 and 1
    float tr_max_inv = 1.0 / 4500;
    float roll_out_high = plane.channel_roll->get_control_in();
    float roll_out = roll_out_high * tr_max_inv;
    float pitch_out_high = plane.channel_pitch->get_control_in();
    float pitch_out = pitch_out_high * tr_max_inv;
    float yaw_out_high = plane.channel_rudder->get_control_in();
    float yaw_out = yaw_out_high * tr_max_inv;
    float throttle_control_high = plane.channel_throttle->get_control_in();
    float throttle_control = 0.01 * throttle_control_high;

    rtU_->cmd.roll = roll_out;
    rtU_->cmd.pitch = pitch_out;
    rtU_->cmd.yaw = yaw_out;
    rtU_->cmd.thr = throttle_control;

    // get rangefinder distance in cm (if sensor with orientation is avail.)
    // int RNGFND_num_sensors = plane.rangefinder.num_sensors();
    rtU_->measure.rangefinder[0] = plane.rangefinder.distance_cm_orient((Rotation)1);
    rtU_->measure.rangefinder[1] = plane.rangefinder.distance_cm_orient((Rotation)2);
    rtU_->measure.rangefinder[2] = plane.rangefinder.distance_cm_orient((Rotation)3);
    rtU_->measure.rangefinder[3] = plane.rangefinder.distance_cm_orient((Rotation)4);
    rtU_->measure.rangefinder[4] = plane.rangefinder.distance_cm_orient((Rotation)5);
    rtU_->measure.rangefinder[5] = plane.rangefinder.distance_cm_orient((Rotation)6);

    // get angular velocity in rad/s
    Vector3f angular_velocity_Kb = plane.ahrs.get_gyro();

    rtU_->measure.omega_Kb[0] = angular_velocity_Kb[0];
    rtU_->measure.omega_Kb[1] = angular_velocity_Kb[1];
    rtU_->measure.omega_Kb[2] = angular_velocity_Kb[2];

    // get acceleration in body-fixed-frame
    Vector3f acc_BFR = plane.ahrs.get_accel() - plane.ahrs.get_accel_bias();
    rtU_->measure.a_bg[0] = acc_BFR.x;
    rtU_->measure.a_bg[1] = acc_BFR.y;
    rtU_->measure.a_bg[2] = acc_BFR.z;

    // get vehicle attitude in quaternions [w x y z]
    Quaternion attitude_vehicle_quat;
    if (!plane.ahrs.get_quaternion(attitude_vehicle_quat))
    {
        attitude_vehicle_quat[0] = 1;
        attitude_vehicle_quat[1] = 0;
        attitude_vehicle_quat[2] = 0;
        attitude_vehicle_quat[3] = 0;
    }

    rtU_->measure.q_bg[0] = attitude_vehicle_quat[0];
    rtU_->measure.q_bg[1] = attitude_vehicle_quat[1];
    rtU_->measure.q_bg[2] = attitude_vehicle_quat[2];
    rtU_->measure.q_bg[3] = attitude_vehicle_quat[3];

    // get alternative attitude representation: euler angles in radian
    rtU_->measure.EulerAngles[0] = plane.ahrs.roll;
    rtU_->measure.EulerAngles[1] = plane.ahrs.pitch;
    rtU_->measure.EulerAngles[2] = plane.ahrs.yaw;

    // get acceleration in NED coordinate system
    Vector3f acc_NED = plane.ahrs.get_accel_ef_blended();
    rtU_->measure.a_Kg[0] = acc_NED.x;
    rtU_->measure.a_Kg[1] = acc_NED.y;
    rtU_->measure.a_Kg[2] = acc_NED.z;

    // get velocity in NED coordinate system
    Vector3f velocity_NED;
    if (!plane.ahrs.get_velocity_NED(velocity_NED))
    {
        velocity_NED[0] = 0;
        velocity_NED[1] = 0;
        velocity_NED[2] = 0;
    }

    rtU_->measure.V_Kg[0] = velocity_NED[0];
    rtU_->measure.V_Kg[1] = velocity_NED[1];
    rtU_->measure.V_Kg[2] = velocity_NED[2];

    // get airspeed estimate from airspeed sensor
    float airspeed_estimate;
    if (!plane.ahrs.airspeed_estimate(airspeed_estimate))
        airspeed_estimate = 0;
    rtU_->measure.airspeed = airspeed_estimate;   


    // get position in NED coordinate system relative to EKF/AHRS origin
    Vector3f position_NED;
    if (!plane.ahrs.get_relative_position_NED_origin(position_NED))
    {
        position_NED[0] = 0;
        position_NED[1] = 0;
        position_NED[2] = 0;
    }

    rtU_->measure.s_Kg[0] = position_NED[0];
    rtU_->measure.s_Kg[1] = position_NED[1];
    rtU_->measure.s_Kg[2] = position_NED[2];

    // get the absolute position in WGS84 coordinate system
    rtU_->measure.lla[0] = plane.current_loc.lat;
    rtU_->measure.lla[1] = plane.current_loc.lng;
    rtU_->measure.lla[2] = plane.current_loc.alt;

    // copy all radio control inputs into the input structure
    for (int i = 0; i < 16; i++)
    {
        rtU_->cmd.RC_pwm[i] = plane.g2.rc_channels.channel(i)->get_radio_in();
    }

    // assign or update waypoints
    // overwrite all custom controller waypoints with 5m above home position
    for (int k = 0; k < max_num_of_matlab_waypoints; k++)
    {
        rtU_->cmd.waypoints[4 * k] = 0.0f;
        rtU_->cmd.waypoints[4 * k + 1] = 0.0f;
        rtU_->cmd.waypoints[4 * k + 2] = -5.0f;
        rtU_->cmd.waypoints[4 * k + 3] = 0.0f;
    }

    int wp_count = 0;
    // start with index j=1 because 1st Ardupilot waypoint is always home position
    for (int j = 1; (j < max_num_of_ardupilot_waypoints) && (j <= numberOfNavCommands); j++)
    {
        // assign only waypoints that are no "ghost waypoints", see declaration of waypoints
        if (abs(waypoints[j][0]) + abs(waypoints[j][1]) + abs(waypoints[j][2]) >= 0.01f)
        {
            rtU_->cmd.waypoints[4 * wp_count] = waypoints[j][0] * 0.01f;     // convert cm to m
            rtU_->cmd.waypoints[4 * wp_count + 1] = waypoints[j][1] * 0.01f; // convert cm to m
            rtU_->cmd.waypoints[4 * wp_count + 2] = waypoints[j][2] * 0.01f; // convert cm to m
            rtU_->cmd.waypoints[4 * wp_count + 3] = waypoints[j][3];         // target velocity in m/s
            wp_count++;
        }
        if (wp_count >= max_num_of_matlab_waypoints)
        {
            // if the maximum number of waypoints that can be send to the matlab controler is reached break
            break;
        }
    }
    rtU_->cmd.num_waypoints = wp_count;           // setting the actual number of valid waypoints
    rtU_->cmd.mission_change = updated_waypoints; // setting the waypoints updated flag
    updated_waypoints = false;

    // run a step in the custom controller generated code
    // custom_controller.step();

    uint16_t bytes_avail = 0;
    uint16_t bytes_read = 0;
    bool bytes_ok = false;
    bool header_ok = false;

    // Receive actuator commands and logs
    bytes_avail = fc_uart->available();
    if (bytes_avail > 3)
    {

        uint8_t header[3] = {0, 0, 0};
        header[0] = fc_uart->read();
        header[1] = fc_uart->read();
        header[2] = fc_uart->read();

        if ((header[0] == 97) && (header[1] == 98) && (header[2] == 99))
        {
            header_ok = true;
            uint8_t command_buffer[200];
            memset(command_buffer, 0, sizeof(command_buffer));

            bytes_read = fc_uart->read(command_buffer, sizeof(command_buffer));

            if (bytes_read == 110)
            {
                uint16_t CRC, CRC_RECV;
                CRC = crc_calculate(command_buffer, sizeof(ExtY));
                memcpy(&CRC_RECV, &(command_buffer[108]), sizeof(CRC_RECV));

                // if checksum is correct, copy buffer into output struct
                if (CRC == CRC_RECV)
                {
                    bytes_ok = true;
                    memcpy(&custom_controller.rtY, command_buffer, sizeof(ExtY));
                }
            }
        }
    }
    fc_uart->discard_input();

    if (bytes_ok == false)
        missed_frames++;

    // Send inputs and measurements
    uint16_t CRC_SEND = crc_calculate((uint8_t *)&(custom_controller.rtU), sizeof(ExtU));
    fc_uart->print("abc");
    fc_uart->write((uint8_t *)&(custom_controller.rtU), sizeof(ExtU));
    fc_uart->write((uint8_t *)&(CRC_SEND), sizeof(CRC_SEND));
    // fc_uart->print("abc");
    fc_uart->flush();

    // get controller outputs struct
    ExtY *rtY_ = &(custom_controller.rtY);

    // send controller outputs to channels and set PWMs
    for (uint8_t i = 0; i < 8; i++)
    {
        SRV_Channel::Aux_servo_function_t function_i = (SRV_Channel::Aux_servo_function_t)rtY_->function_channels[i];

        if (function_i == SRV_Channel::Aux_servo_function_t::k_none)
        {
            continue;
        }
        if (function_i == SRV_Channel::Aux_servo_function_t::k_throttle)
        {
            if (!hal.util->get_soft_armed())
            {
                if (plane.arming.arming_required() == AP_Arming::Required::YES_ZERO_PWM)
                {
                    SRV_Channels::set_output_limit(function_i, SRV_Channel::Limit::ZERO_PWM);
                }
                else
                {
                    SRV_Channels::set_output_norm(function_i, 0);
                }
            }
            else
            {
                SRV_Channels::set_output_norm(function_i, rtY_->channels[i]);
            }
        }
        else
        {
            SRV_Channels::set_output_norm(function_i, rtY_->channels[i]);
        }
    }

    uint32_t modecustom_duration_us = AP_HAL::micros() - time_beg;

    // Log the execution time and report the maximum
    static uint32_t modecustom_max_us = 0;
    if (modecustom_duration_us > modecustom_max_us)
        modecustom_max_us = modecustom_duration_us;

    // log data
    AP::logger().Write(
        "ML", "TimeUS,v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12,v13,v14,v15",
        "Qfffffffffffffff",
        AP_HAL::micros64(),
        (double)modecustom_duration_us,
        //(double)rtY.logs[0],
        (double)rtY_->logs[1],
        (double)rtY_->logs[2],
        (double)rtY_->logs[3],
        (double)rtY_->logs[4],
        (double)rtY_->logs[5],
        (double)rtY_->logs[6],
        (double)rtY_->logs[7],
        (double)rtY_->logs[8],
        (double)rtY_->logs[9],
        (double)rtY_->logs[10],
        (double)rtY_->logs[11],
        (double)rtY_->logs[12],
        (double)rtY_->logs[13],
        (double)rtY_->logs[14]);

    // log attitude
    AP::logger().Write("QUAT", "TimeUS,q0,q1,q2,q3", "Qffff",
                       AP_HAL::micros64(),
                       (double)attitude_vehicle_quat[0],
                       (double)attitude_vehicle_quat[1],
                       (double)attitude_vehicle_quat[2],
                       (double)attitude_vehicle_quat[3]);

    static uint16_t counter;
    static uint32_t last_t, last_print;
    uint32_t now = AP_HAL::micros();

    if (last_t == 0)
    {
        last_t = now;
        return;
    }
    last_t = now;

    counter++;

    // display some diagnostics every five second
    if ((now - last_print >= 5e6) || (rtU_->cmd.mission_change == 1) /* 5e6 us -> 5.0 hz */)
    {

        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Bytes avail: %d, Header: %d, Bytes read: %d, Missed Frames: %d \n", bytes_avail, header_ok, bytes_read, missed_frames);
        missed_frames = 0;

        // send rangefinder distance in cm for debugging
        /*GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "%d: distance_cm %f %f %f %f %f %f \n",
            RNGFND_num_sensors,
            rangefinder_dist[0], rangefinder_dist[1], rangefinder_dist[2],
            rangefinder_dist[3], rangefinder_dist[4], rangefinder_dist[5]);*/

        // send PWM outputs for each servo for debugging
        /*GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Servo PWM: %f %f %f %f %f %f %f %f \n",
            rtY_->channels[0], rtY_->channels[1], rtY_->channels[2], rtY_->channels[3],
            rtY_->channels[4], rtY_->channels[5], rtY_->channels[6], rtY_->channels[7]);
            */

        // send attitude of vehicle for for debugging
        // gcs().send_text(MAV_SEVERITY_DEBUG, "test %5.3f %5.3f %5.3f %5.3f", (double)attitude_vehicle_quat[0], (double)attitude_vehicle_quat[1], attitude_vehicle_quat[2], attitude_vehicle_quat[3]);

        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Time in us: %d, max: %d Logs: %f %f %f %f %f %f %f %f %f  \n", (int)modecustom_duration_us, (int)modecustom_max_us,
                 rtY_->logs[9], rtY_->logs[10], rtY_->logs[11], rtY_->logs[12], rtY_->logs[13], rtY_->logs[14], rtY_->logs[0], rtY_->logs[1], rtY_->logs[2]);

        last_print = now;
        counter = 0;
    }
}

void ModeCustom::add_waypoint(uint16_T index, Vector3f location)
{ // adding a waypoint to the struct, this function is called when a new waypoint is added via AP_Mission
    waypoints[index][0] = location.x;
    waypoints[index][1] = location.y;
    waypoints[index][2] = -location.z;
    waypoints[index][3] = 0.0f;
    numberOfNavCommands = index;
}

void ModeCustom::add_speed(uint16_T index, float V_k)
{ // adding a new 'waypoint' which contains only the speed information set via AP_Mission
    if (abs(waypoints[index - 1][0] + waypoints[index - 1][1] + waypoints[index - 1][2]) >= 0.1f)
    {
        waypoints[index][0] = 0.0f;
        waypoints[index][1] = 0.0f;
        waypoints[index][2] = 0.0f;
        waypoints[index - 1][3] = V_k;
    }
    numberOfNavCommands = index;
}