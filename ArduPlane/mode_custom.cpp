#include "mode.h"
#include "Plane.h"
#include <AP_Common/MatlabController.h>

bool ModeCustom::_enter()
{
    custom_controller.initialize(); 

    // init custom logging
    log_setup(log_config);

    return true;
}


void ModeCustom::update()
{

    uint32_t time_beg = AP_HAL::micros();

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
    Vector3f angular_velocity_Kb = plane.ahrs.get_gyro();

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

    Vector3f position_NED;
    //if(!plane.ahrs.get_relative_position_NED_home(position_NED))
    if(!plane.ahrs.get_relative_position_NED_origin(position_NED))
    {
        position_NED[0] = 0;
        position_NED[1] = 0;
        position_NED[2] = 0;
    }


    // assign commanded and measured values to controller inputs struct
    ExtU rtU_;

    rtU_.cmd.roll  = roll_out;
    rtU_.cmd.pitch = pitch_out;
    rtU_.cmd.yaw   = yaw_out;
    rtU_.cmd.thr   = throttle_control;
     for (int i=0;i<16;i++) {
        rtU_.cmd.RC_pwm[i] = plane.g2.rc_channels.channel(i)->get_radio_in();
    }

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

    // assign or update waypoints
    // overwrite all custom controller waypoints with 5m above home position
    for (int k=0;k<max_num_of_matlab_waypoints;k++){
        rtU_.cmd.waypoints[4*k]   = 0.0f;
        rtU_.cmd.waypoints[4*k+1] = 0.0f;
        rtU_.cmd.waypoints[4*k+2] = -5.0f;
        rtU_.cmd.waypoints[4*k+3] = 0.0f;
    }
    int wp_count=0;
    // start with index j=1 because 1st Ardupilot waypoint is always home position
    for (int j=1;(j<max_num_of_ardupilot_waypoints)&&(j<numberOfNavCommands);j++){
        // assign only waypoints that are no "ghost waypoints", see declaration of waypoints
        if (abs(waypoints[j][0]) + abs(waypoints[j][1]) + abs(waypoints[j][2]) >= 0.01f){
            rtU_.cmd.waypoints[4*wp_count]   = waypoints[j][0]*0.01f; // convert cm to m
            rtU_.cmd.waypoints[4*wp_count+1] = waypoints[j][1]*0.01f; // convert cm to m
            rtU_.cmd.waypoints[4*wp_count+2] = waypoints[j][2]*0.01f; // convert cm to m
            rtU_.cmd.waypoints[4*wp_count+3] = waypoints[j][3]; // target velocity in m/s
            wp_count++;
        }
        if (wp_count>=max_num_of_matlab_waypoints){
            // if the maximum number of waypoints that can be send to the matlab controler is reached break
            break;
        }
    }
    rtU_.cmd.num_waypoints = wp_count;  //setting the actual number of valid waypoints
    rtU_.cmd.mission_change = updated_waypoints; // setting the waypoints updated flag
    updated_waypoints = false;

    // get controller outputs struct
    // ToDo: Use Pointer for rtU_ and rtY_ because coping all data in every step is NOT necessary, this will save a lot of memory for the waypoints and some cpu time.
    custom_controller.rtU = rtU_;
    custom_controller.step(); //run a step in controller.
    ExtY rtY_ = custom_controller.rtY;

    // log signals
    for (int i=0;i<num_log_batches;i++) {
        write_log_custom(batch_name_full[i], label_full[i],
            &custom_controller.rtY.logs[log_signal_idx_cumsum[i]],
            log_config[i].num_signals);
    }

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

    uint32_t modecustom_duration_us = AP_HAL::micros() - time_beg;

    // Log the execution time and report the maximum
    static uint32_t modecustom_max_us = 0;
    if(modecustom_duration_us > modecustom_max_us) 
        modecustom_max_us = modecustom_duration_us;
    AP::logger().Write(
        "MLPM", "TimeUS,dtUS",
        "Qf",
        AP_HAL::micros64(),
        (double)modecustom_duration_us);    
    

    if ((now - last_print >= 1e6) || (rtU_.cmd.mission_change == 1) /* 1000 ms : 1.0 hz */ ) {

        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Time in us: %d, max: %d  \n", (int)modecustom_duration_us, (int)modecustom_max_us);
            
        last_print = now;
        counter = 0;
    }


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

void ModeCustom::get_log_label(int batch_number, char *label) {
    memcpy(label,&(label_full[batch_number-1]),label_length[batch_number-1]+1);
    label[label_length[batch_number-1]]=0;
};

void ModeCustom::get_log_batch_name(int batch_number, char *name) {
    memcpy(name,&(batch_name_full[batch_number-1]),batch_name_length[batch_number-1]+1);
    name[batch_name_length[batch_number-1]]=0;
};
