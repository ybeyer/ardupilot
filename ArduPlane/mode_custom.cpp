#include "mode.h"
#include "Plane.h"
#include <AP_Common/MatlabController.h>
#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>

// Check if MatlabControllerClass contains parameters
//  - if MatlabControllerClass was postprocessed for tunable parameters:
//      - 'MatlabController.h' defines MODE_CUSTOM_VAR_INFO
//      - 'ModeCustomParams.cpp' defines 'const AP_Param::GroupInfo ModeCustom::var_info'
#ifndef MODE_CUSTOM_VAR_INFO
#define MODE_CUSTOM_VAR_INFO
const AP_Param::GroupInfo ModeCustom::var_info[] = {
	AP_GROUPEND
};
#endif

#ifdef Mode_Custom_Use_External_Controller
    // Matek H743-MINI: 1 = TX7, 2 = TX1, 3 = TX2, 6 = TX4, 7 = TX6
    AP_HAL::UARTDriver *fc_uart = hal.serial(2);
    uint16_t bytes_avail = 0;
    uint16_t bytes_read = 0;
    uint16_t missed_frames = 0;
#endif


#ifdef CUSTOM_MATLAB_OUTPUT
// constructor
ModeCustom::ModeCustom(void) : Mode(), socket_debug(true)
{
}
#endif


bool ModeCustom::_enter()
{
    #ifdef Mode_Custom_Use_External_Controller
        fc_uart->begin(6000000, sizeof(ExtU), sizeof(ExtY));
    #endif

    custom_controller.initialize(); 
    updated_waypoints = true;

    // init custom logging
    log_setup(log_config);

    return true;
}


void ModeCustom::update()
{

    uint32_t time_total = AP_HAL::micros();

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

    // int RNGFND_num_sensors = plane.rangefinder.num_sensors();
    
    // get measured inputs
    /* Info: gyro scaling is hard coded based on AP_InertialSensor::register_gyro in AP_InertialSensor.cpp.
    The scaling is applied in AP_InertialSensor_Backend::_notify_new_gyro_raw_sample in
    AP_InertialSensor_Backend.cpp. However, the variable gyro_filtered is overwritten during filtering.
    It seems that there is no non-filtered scaled angular velocity available as member variable.
    That is why the scaling is applied here.) */
    Vector3f Omega_Kb_raw = AP::ins().get_raw_gyro() / (INT16_MAX/radians(2000));

    Quaternion attitude_vehicle_quat;
    if(!plane.ahrs.get_quaternion(attitude_vehicle_quat))
    {
        attitude_vehicle_quat[0] = 1;
        attitude_vehicle_quat[1] = 0;
        attitude_vehicle_quat[2] = 0;
        attitude_vehicle_quat[3] = 0;
    }

    Vector3f acc_NED = plane.ahrs.get_accel_ef_blended();

    Vector3f velocity_NED;
    if(!plane.ahrs.get_velocity_NED(velocity_NED))
    {
        velocity_NED[0] = 0;
        velocity_NED[1] = 0;
        velocity_NED[2] = 0;
    }

    // get airspeed estimate from airspeed sensor
    float airspeed_estimate;
    if (!plane.ahrs.airspeed_estimate(airspeed_estimate)) {
        airspeed_estimate = 0;
    }    

    // get acceleration in body-fixed-frame
    Vector3f acc_FRD = AP::ins().get_raw_accel() - AP::ins().get_accel_offsets();

    Vector3f position_NED;
    //if(!plane.ahrs.get_relative_position_NED_home(position_NED))
    if(!plane.ahrs.get_relative_position_NED_origin(position_NED))
    {
        position_NED[0] = 0;
        position_NED[1] = 0;
        position_NED[2] = 0;
    }


    // assign commanded and measured values to controller inputs struct
    ExtU *rtU_ = &(custom_controller.rtU);
    memset(rtU_, 0, sizeof(ExtU));

    rtU_->cmd.roll  = roll_out;
    rtU_->cmd.pitch = pitch_out;
    rtU_->cmd.yaw   = yaw_out;
    rtU_->cmd.thr   = throttle_control;
     for (int i=0;i<16;i++) {
        rtU_->cmd.RC_pwm[i] = plane.g2.rc_channels.channel(i)->get_radio_in();
    }

    rtU_->measure.omega_Kb[0] = Omega_Kb_raw[0];
    rtU_->measure.omega_Kb[1] = Omega_Kb_raw[1];
    rtU_->measure.omega_Kb[2] = Omega_Kb_raw[2];
    rtU_->measure.q_bg[0] = attitude_vehicle_quat[0];
    rtU_->measure.q_bg[1] = attitude_vehicle_quat[1];
    rtU_->measure.q_bg[2] = attitude_vehicle_quat[2];
    rtU_->measure.q_bg[3] = attitude_vehicle_quat[3];
    rtU_->measure.EulerAngles[0] = plane.ahrs.roll;
    rtU_->measure.EulerAngles[1] = plane.ahrs.pitch;
    rtU_->measure.EulerAngles[2] = plane.ahrs.yaw;
    rtU_->measure.a_Kg[0] = acc_NED[0];
    rtU_->measure.a_Kg[1] = acc_NED[1];
    rtU_->measure.a_Kg[2] = acc_NED[2];
    rtU_->measure.a_Kb[0] = acc_FRD[0];
    rtU_->measure.a_Kb[1] = acc_FRD[1];
    rtU_->measure.a_Kb[2] = acc_FRD[2];
    rtU_->measure.V_Kg[0] = velocity_NED[0];
    rtU_->measure.V_Kg[1] = velocity_NED[1];
    rtU_->measure.V_Kg[2] = velocity_NED[2];
    rtU_->measure.airspeed = airspeed_estimate;   
    rtU_->measure.s_Kg[0] = position_NED[0];
    rtU_->measure.s_Kg[1] = position_NED[1];
    rtU_->measure.s_Kg[2] = position_NED[2];
    rtU_->measure.lla[0] = plane.current_loc.lat;
    rtU_->measure.lla[1] = plane.current_loc.lng;
    rtU_->measure.lla[2] = plane.current_loc.alt;
    rtU_->measure.rangefinder[0] = rangefinder_dist[0];
    rtU_->measure.rangefinder[1] = rangefinder_dist[1];
    rtU_->measure.rangefinder[2] = rangefinder_dist[2];
    rtU_->measure.rangefinder[3] = rangefinder_dist[3];
    rtU_->measure.rangefinder[4] = rangefinder_dist[4];
    rtU_->measure.rangefinder[5] = rangefinder_dist[5];

    // assign or update waypoints
    // overwrite all custom controller waypoints with 5m above home position
    for (int k=0;k<max_num_of_matlab_waypoints;k++){
        rtU_->cmd.waypoints[4*k]   = 0.0f;
        rtU_->cmd.waypoints[4*k+1] = 0.0f;
        rtU_->cmd.waypoints[4*k+2] = -5.0f;
        rtU_->cmd.waypoints[4*k+3] = 0.0f;
    }
    int wp_count=0;
    // start with index j=1 because 1st Ardupilot waypoint is always home position
    for (uint16_t j = 1;(j<max_num_of_ardupilot_waypoints)&&(j<=numberOfNavCommands);j++){
        // assign only waypoints that are no "ghost waypoints", see declaration of waypoints
        if (abs(waypoints[j][0]) + abs(waypoints[j][1]) + abs(waypoints[j][2]) >= 0.01f){
            rtU_->cmd.waypoints[4*wp_count]   = waypoints[j][0]*0.01f; // convert cm to m
            rtU_->cmd.waypoints[4*wp_count+1] = waypoints[j][1]*0.01f; // convert cm to m
            rtU_->cmd.waypoints[4*wp_count+2] = waypoints[j][2]*0.01f; // convert cm to m
            rtU_->cmd.waypoints[4*wp_count+3] = waypoints[j][3]; // target velocity in m/s
            wp_count++;
        }
        if (wp_count>=max_num_of_matlab_waypoints){
            // if the maximum number of waypoints that can be send to the matlab controler is reached break
            break;
        }
    }
    rtU_->cmd.num_waypoints = wp_count;  //setting the actual number of valid waypoints
    rtU_->cmd.mission_change = updated_waypoints; // setting the waypoints updated flag
    updated_waypoints = false;

    // get controller outputs struct
    uint32_t time_step = AP_HAL::micros();
    #ifdef Mode_Custom_Use_External_Controller
        step_external();
    #else
        custom_controller.step(); //run a step in controller.
    #endif
    time_step = AP_HAL::micros() - time_step;
    ExtY *rtY_ = &(custom_controller.rtY);

    // DEBUGGING:
    // Send all inputs of custom controller to Simulink (uncomment line 3 in mode.h)
    // Check byte alignment/padding in Simulink, while receiving (e.g. 4)
    #ifdef CUSTOM_MATLAB_OUTPUT
        socket_debug.sendto(rtU_, sizeof(ExtU), _debug_address, _debug_port); 
    #endif

    // log signals
    uint32_t time_log = AP_HAL::micros();
    for (int i=0;i<num_log_batches;i++) {
        write_log_custom(batch_name_full[i], label_full[i],
            &custom_controller.rtY.logs[log_signal_idx_cumsum[i]],
            log_config[i].num_signals);
    }
    time_log = AP_HAL::micros() - time_log;

    // send controller outputs to channels and set PWMs
    bool is_passthrough;
    bool is_servo;
    float u_norm = 0.0f;
    uint16_t zero_pwm = 0;
    SRV_Channel::Aux_servo_function_t function_i;
    SRV_Channel *c;
    for (uint8_t i=0; i<16; i++) {
        is_passthrough = rtY_->function_channels[i]==999;
        c = SRV_Channels::srv_channel(i);
        if (is_passthrough) {
            is_servo = c->is_servo();
            u_norm = rtY_->channels[i];
            if (c->get_reversed()) {
                if (is_servo) {
                    u_norm = -u_norm;
                } else {
                    u_norm = 1.0f - u_norm;
                }
            }
        } else {
            function_i = (SRV_Channel::Aux_servo_function_t)rtY_->function_channels[i];
            if(function_i == SRV_Channel::Aux_servo_function_t::k_none){
                continue;  
            }
        }
        // similar to servos.cpp, l. 714
        if (!hal.util->get_soft_armed() && !c->is_servo()) {
            if (plane.arming.arming_required() == AP_Arming::Required::YES_ZERO_PWM) {
                if (is_passthrough) {
                    //zero_pwm = c->get_limit_pwm(SRV_Channel::Limit::ZERO_PWM);
                    c->set_output_pwm(zero_pwm);
                } else {
                    SRV_Channels::set_output_limit(function_i, SRV_Channel::Limit::ZERO_PWM);
                }
            } else {
                if (is_passthrough) {
                    c->set_output_norm(0);
                } else {
                    SRV_Channels::set_output_norm(function_i, 0);
                }
            }
        } else {
            if (is_passthrough) {
                c->set_output_norm(u_norm);
            } else {
                SRV_Channels::set_output_norm(function_i, rtY_->channels[i]);
            }
        }
    }
    

    time_total = AP_HAL::micros() - time_total;
    static uint32_t modecustom_max_us = 0;
    if(time_total > modecustom_max_us){modecustom_max_us = time_total;}

    // Log the execution times
    AP::logger().Write(
        "MLPM", "TimeUS,TimeTotalUS,TimeStepUS,TimeLogUS,TimeMaxUS",
        "Qffff",
        AP_HAL::micros64(),
        (double)time_total, (double)time_step, (double)time_log, (double)modecustom_max_us );

}


void ModeCustom::add_waypoint(uint16_T index,Vector3f location){ // adding a waypoint to the struct, this function is called when a new waypoint is added via AP_Mission
        waypoints[index][0] = location.x;
        waypoints[index][1] = location.y;
        waypoints[index][2] = -location.z;
        waypoints[index][3] = 0.0f;
        numberOfNavCommands = index;
}

void ModeCustom::add_speed(uint16_T index, float V_k){ //adding a new 'waypoint' which contains only the speed information set via AP_Mission
    if(abs(waypoints[index-1][0] + waypoints[index-1][1] + waypoints[index-1][2]) >= 0.1f){
        waypoints[index][0] = 0.0f;
        waypoints[index][1] = 0.0f;
        waypoints[index][2]     = 0.0f;
        waypoints[index-1][3] = V_k;
    }
    numberOfNavCommands = index;
}

void ModeCustom::log_setup(const logConfigBus log_config_in[]) {
    set_log_batch_names(log_config_in);
    set_log_labels(log_config_in);
    set_log_signal_idx_cumsum(log_config_in);
};

void ModeCustom::set_log_batch_names(const logConfigBus log_config_in[]) {
    for (int i=0;i<num_log_batches;i++) {
        memcpy(&(batch_name_full[i][0]), &(log_config_in[i].batch_name), max_batch_name_length);
        batch_name_length[i]=max_batch_name_length;
        for (int j=0;j<max_batch_name_length;j++) {
            if (batch_name_full[i][j] == 1) {
                batch_name_length[i] --;
            }
        }
    }
};

void ModeCustom::set_log_labels(const logConfigBus log_config_in[]){
    signal_name_t current_name_int;
    int signal_name_length;
    for (int i=0;i<num_log_batches;i++) {
        const uint8_t *log_names_in = log_config_in[i].signal_names;
        memcpy(&(label_full[i][0]), &("TimeUS"), 6);
        label_length[i]=6;
        for (int j=0;j<log_config_in[i].num_signals;j++) {
            signal_name_length = max_signal_name_length;
            memcpy(&(label_full[i][label_length[i]]),&(","),1);
            label_length[i] ++;
            extract_one_signal_name(log_names_in, j+1, current_name_int);
            for (int k=0;k<max_signal_name_length;k++) {
                if (current_name_int[k]==1) {
                    signal_name_length --;
                }
            }
            memcpy(&(label_full[i][label_length[i]]),&current_name_int,signal_name_length);
            label_length[i] += signal_name_length;
        }
    }
};

void ModeCustom::set_log_signal_idx_cumsum(const logConfigBus log_config_in[]){
    log_signal_idx_cumsum[0] = 0;
    for (int i=1;i<num_log_batches;i++) {
        log_signal_idx_cumsum[i] = log_signal_idx_cumsum[i-1] + log_config_in[i-1].num_signals;
    }
};

void ModeCustom::write_log_custom(const char *name, const char *labels, float *sf, int size) {
    double s[size];
    for (int i=0; i<size; i++) {
        s[i] = (double)sf[i];
    }
    if (size==0) {
        return;
    } else if (size==1) {
        AP::logger().Write(name, labels,"Qf",AP_HAL::micros64(),s[0]);
    } else if (size==2) {
        AP::logger().Write(name, labels,"Qff",AP_HAL::micros64(),s[0],s[1]);
    } else if (size==3) {
        AP::logger().Write(name, labels,"Qfff",AP_HAL::micros64(),s[0],s[1],s[2]);
    } else if (size==4) {
        AP::logger().Write(name, labels,"Qffff",AP_HAL::micros64(),s[0],s[1],s[2],s[3]);
    } else if (size==5) {
        AP::logger().Write(name, labels,"Qfffff",AP_HAL::micros64(),s[0],s[1],s[2],s[3],s[4]);
    } else if (size==6) {
        AP::logger().Write(name, labels,"Qffffff",AP_HAL::micros64(),s[0],s[1],s[2],s[3],s[4],s[5]);
    } else if (size==7) {
        AP::logger().Write(name, labels,"Qfffffff",AP_HAL::micros64(),s[0],s[1],s[2],s[3],s[4],s[5],s[6]);
    } else if (size==8) {
        AP::logger().Write(name, labels,"Qffffffff",AP_HAL::micros64(),s[0],s[1],s[2],s[3],s[4],s[5],s[6],s[7]);
    } else if (size==9) {
        AP::logger().Write(name, labels,"Qfffffffff",AP_HAL::micros64(),s[0],s[1],s[2],s[3],s[4],s[5],s[6],s[7],s[8]);
    } else if (size==10) {
        AP::logger().Write(name, labels,"Qffffffffff",AP_HAL::micros64(),s[0],s[1],s[2],s[3],s[4],s[5],s[6],s[7],s[8],s[9]);
    } else if (size==11) {
        AP::logger().Write(name, labels,"Qfffffffffff",AP_HAL::micros64(),s[0],s[1],s[2],s[3],s[4],s[5],s[6],s[7],s[8],s[9],s[10]);
    } else if (size==12) {
        AP::logger().Write(name, labels,"Qffffffffffff",AP_HAL::micros64(),s[0],s[1],s[2],s[3],s[4],s[5],s[6],s[7],s[8],s[9],s[10],s[11]);
    } else if (size==13) {
        AP::logger().Write(name, labels,"Qfffffffffffff",AP_HAL::micros64(),s[0],s[1],s[2],s[3],s[4],s[5],s[6],s[7],s[8],s[9],s[10],s[11],s[12]);
    } else if (size==14) {
        AP::logger().Write(name, labels,"Qffffffffffffff",AP_HAL::micros64(),s[0],s[1],s[2],s[3],s[4],s[5],s[6],s[7],s[8],s[9],s[10],s[11],s[12],s[13]);
    } else if (size==15) {
        AP::logger().Write(name, labels,"Qfffffffffffffff",AP_HAL::micros64(),s[0],s[1],s[2],s[3],s[4],s[5],s[6],s[7],s[8],s[9],s[10],s[11],s[12],s[13],s[14]);
    }
};

void ModeCustom::extract_one_signal_name(const uint8_t log_names_int[], int number, signal_name_t &log_name){
    int idx = (number-1)*max_signal_name_length;
    for (int i=0;i<max_signal_name_length;i++) {
        log_name[i] = log_names_int[idx+i];
    }
};

#ifdef Mode_Custom_Use_External_Controller
void ModeCustom::step_external(){
    bytes_avail = 0;
    bytes_read = 0;
    bool bytes_ok = false;

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
    // message identifier
    fc_uart->print("abc");
    fc_uart->write((uint8_t *)&(custom_controller.rtU), sizeof(ExtU));
    fc_uart->write((uint8_t *)&(CRC_SEND), sizeof(CRC_SEND));
    fc_uart->flush();
};
#endif
