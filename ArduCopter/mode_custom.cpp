#include "Copter.h"
#include <AP_Motors/AP_MotorsMatrix.h>
#include <GCS_MAVLink/GCS.h>

// Check if MatlabControllerClass contains parameters
// If MatlabControllerClass was postprocessed for tunable parameters, use AP_SUBGROUPINFO
#ifndef MODE_CUSTOM_VAR_INFO
#define MODE_CUSTOM_VAR_INFO
const AP_Param::GroupInfo ModeCustom::var_info[] = {
	AP_GROUPEND
};
#else
const AP_Param::GroupInfo ModeCustom::var_info[] = {
  AP_SUBGROUPINFO(custom_controller, "", 0, ModeCustom, MatlabControllerClass),
	AP_GROUPEND
};
#endif


#ifdef CUSTOM_MATLAB_OUTPUT
// constructor
ModeCustom::ModeCustom(void) : Mode(), socket_debug(true)
{
}
#endif

// Init Matlab controller inputs and outputs
ExtU rtU_;
//memset(&rtU_, 0, sizeof(rtU_));
ExtY rtY_;

// Function for hardcoding changes to MATLABs cntrl struct.
// Values can be accessed in the same fashion as in MATLAB, e.g.:
//     cntrl.sample_time = 42;
void ModeCustom::override_cntrl_params()
{

}

// init custom flight mode
bool ModeCustom::init(bool ignore_checks)
{
    override_cntrl_params();
    // initialize yaw to measured value
    const AP_AHRS_View &ahrs_ = attitude_control->get_ahrs();
    Quaternion attitude_vehicle_quat;
    ahrs_.get_quat_body_to_ned(attitude_vehicle_quat);
    yawInit = atan2f(copter.simple_sin_yaw,copter.simple_cos_yaw);
    AP_Mission *mission = AP::mission();
    mission->updated_waypoints = true;
    // fill waypoints array with zeros
    memset(rtU_.cmd.waypoints, 0, sizeof(rtU_.cmd.waypoints));
    // initialize position to measured value
    Vector3f position_NED;
    if (!ahrs_.get_relative_position_NED_home(position_NED)){
        position_NED[0] = 0;
        position_NED[1] = 0;
        position_NED[2] = 0; 
    }
    sInit[0] = position_NED[0];
    sInit[1] = position_NED[1];
    sInit[2] = position_NED[2];

    // tell the controller to use the initial conditions on the first time step
    custom_controller.initialize();

    // init custom logging
    log_setup(log_config);

    return true;
}

// run custom flight mode
void ModeCustom::run()
{

    uint64_t time_total = AP_HAL::micros();

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
    if (!ahrs_.get_velocity_NED(velocity_NED)) {
        velocity_NED[0] = 0;
        velocity_NED[1] = 0;
        velocity_NED[2] = 0;
    }

    // get measured inputs
    Vector3f Omega_Kb_raw = AP::ins().get_gyro_for_fft() / (INT16_MAX/radians(2000));
    Vector3f Omega_Kb_ntch = AP::ins().get_gyro_ntch();     // gyro filtered by notch filter (only if ML_GYR_HNTCH=1)
    // Vector3f Omega_Kb_f = AP::ins().get_gyro();          // filtered gyro (Static Notches -> Dynamic Notches -> Lowpass (INS_GYRO_FILTER)), Kb
    Vector3f Omega_Kb = AP::ins().get_ml_gyro();            // filtered gyro (ML-Lowpass (INS_ML_GYRO_FILTER and notch (if ML_GYR_HNTCH=1)), Kb
    // Vector3f OmegaML_Kb_f_dt = AP::ins().get_ml_gyro_dt();   // derivative (two-point backward finite difference) of filtered gyro, Kb

    // get acceleration in body-fixed frame
    Vector3f acc_FRD_raw = AP::ins().get_raw_accel() - AP::ins().get_accel_offsets();
    Vector3f acc_FRD = AP::ins().get_ml_accel();            // filtered accel (ML-Lowpass (INS_ML_ACC_FLTER)), Kb
    Vector3f acc_NED_at_IMU = acc_FRD;
    // acceleration in earth-fixed frame
    attitude_vehicle_quat.earth_to_body(acc_NED_at_IMU);    // note: this is the right function although the name is misleading in this case
    // https://ardupilot.org/copter/docs/common-sensor-offset-compensation.html
    Vector3f imu_pos_offset = AP::ins().get_imu_pos_offset(AP::ins().get_first_usable_accel());
    Vector3f acc_NED_at_COG = imu_accel_to_cog_accel(  acc_NED_at_IMU, imu_pos_offset, Omega_Kb,
                                                        ahrs_.get_rotation_body_to_ned() );

    float roll_angle = attitude_vehicle_quat.get_euler_roll();
    float pitch_angle = attitude_vehicle_quat.get_euler_pitch();
    float yaw_angle = attitude_vehicle_quat.get_euler_yaw();
    // Get position relative to the ground in NED
    Vector3f position_NED;
    if (!ahrs_.get_relative_position_NED_home(position_NED)) {
        position_NED[0] = 0;
        position_NED[1] = 0;
        position_NED[2] = 0;
    }
    Vector3f position_NED_origin;
    if (!ahrs_.get_relative_position_NED_origin(position_NED_origin)) {
        position_NED_origin[0] = 0;
        position_NED_origin[1] = 0;
        position_NED_origin[2] = 0;
    }
    float voltage = copter.battery.voltage();


    // To do: spool states are currently based on copy from mode_stabilize
    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero) {
        // Attempting to Land
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }
    // To do: spool states are currently based on copy from mode_stabilize
    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        // To do!
        break;
    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        // To do!
        break;
    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        // To do!
    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        // To do!
        break;
    }

    // assign commanded controller inputs to cmd struct
    rtU_.cmd.roll = roll_out;
    rtU_.cmd.pitch = pitch_out;
    rtU_.cmd.yaw = yaw_out;
    rtU_.cmd.thr = throttle_control;
    rtU_.cmd.s_Kg_init[0] = sInit[0];
    rtU_.cmd.s_Kg_init[1] = sInit[1];
    rtU_.cmd.s_Kg_init[2] = sInit[2];
    rtU_.cmd.yaw_init = yawInit;
    for (int i=0;i<16;i++) {
        rtU_.cmd.RC_pwm[i] = g2.rc_channels.channel(i)->get_radio_in();
    }

    // assign or update waypoints
    AP_Mission *mission = AP::mission();
    if (mission->updated_waypoints){
        // start with index j=1 because 1st Ardupilot waypoint is always home position
        for (int j=0;j<mission->num_wp;j++){
            // assign only waypoints that are no "ghost waypoints", see declaration of waypoints
            rtU_.cmd.waypoints[4*j]   = mission->waypoints[j][0]*0.01f; // convert cm to m
            rtU_.cmd.waypoints[4*j+1] = mission->waypoints[j][1]*0.01f; // convert cm to m
            rtU_.cmd.waypoints[4*j+2] = mission->waypoints[j][2]*0.01f; // convert cm to m
            rtU_.cmd.waypoints[4*j+3] = mission->waypoints[j][3]; // target velocity in m/s
        }
    }
    rtU_.cmd.num_waypoints = mission->num_wp;
    rtU_.cmd.mission_change = mission->updated_waypoints;
    mission->updated_waypoints = false;


    // assign measured controller inputs to measure struct
    rtU_.measure.omega_Kb[0] = Omega_Kb[0];
    rtU_.measure.omega_Kb[1] = Omega_Kb[1];
    rtU_.measure.omega_Kb[2] = Omega_Kb[2];
    rtU_.measure.q_bg[0] = attitude_vehicle_quat.q1;
    rtU_.measure.q_bg[1] = attitude_vehicle_quat.q2;
    rtU_.measure.q_bg[2] = attitude_vehicle_quat.q3;
    rtU_.measure.q_bg[3] = attitude_vehicle_quat.q4;
    rtU_.measure.EulerAngles[0] = roll_angle;
    rtU_.measure.EulerAngles[1] = pitch_angle;
    rtU_.measure.EulerAngles[2] = yaw_angle;
    rtU_.measure.a_Kg[0] = acc_NED_at_COG.x;
    rtU_.measure.a_Kg[1] = acc_NED_at_COG.y;
    rtU_.measure.a_Kg[2] = acc_NED_at_COG.z;
    rtU_.measure.V_Kg[0] = velocity_NED[0];
    rtU_.measure.V_Kg[1] = velocity_NED[1];
    rtU_.measure.V_Kg[2] = velocity_NED[2];
    rtU_.measure.s_Kg[0] = position_NED[0];
    rtU_.measure.s_Kg[1] = position_NED[1];
    rtU_.measure.s_Kg[2] = position_NED[2];
    rtU_.measure.s_Kg_origin[0] = position_NED[0];
    rtU_.measure.s_Kg_origin[1] = position_NED_origin[1];
    rtU_.measure.s_Kg_origin[2] = position_NED_origin[2];
    rtU_.measure.lla[0] = copter.current_loc.lat;
    rtU_.measure.lla[1] = copter.current_loc.lng;
    rtU_.measure.lla[2] = copter.current_loc.alt;
    rtU_.measure.V_bat = voltage;

    // assign motor RPM
    bool is_omega_mot_found = false;
    for (int i=0;i<4;i++) {
        is_omega_mot_found = AP::esc_telem().get_raw_rpm(i,rtU_.measure.omega_mot[i]);
        if (!is_omega_mot_found) {
            rtU_.measure.omega_mot[i] = -1.0f;
         } else {
             rtU_.measure.omega_mot[i] *= 2*3.14159/60;
        }
    }
    
    // run Simulink controller
    custom_controller.rtU = rtU_;
    uint64_t time = AP_HAL::micros64();
    custom_controller.step();
    uint64_t time_step = AP_HAL::micros() - time;
    rtY_ = custom_controller.rtY;

    // DEBUGGING:
    // Send all inputs of custom controller to Simulink (uncomment line 3 in mode.h)
    // Check byte alignment/padding in Simulink, while receiving (e.g. 4)
    #ifdef CUSTOM_MATLAB_OUTPUT
        socket_debug.sendto(&rtU_.measure, sizeof(rtU_), _debug_address, _debug_port);
    #endif

    // log signals
    uint64_t time_log = AP_HAL::micros();
    for (int i=0;i<num_log_batches;i++) {
        write_log_custom(batch_name_full[i], label_full[i],
            &custom_controller.rtY.logs[log_signal_idx_cumsum[i]],
            log_config[i].num_signals, time);
    }
    time_log = AP_HAL::micros() - time_log;

    // set outputs in the same order as Simulink
    for (int i=0;i<8;i++) {
        motors->set_custom_input( i, rtY_.u[i] );
    }

    motors->output_custom();

    time_total = AP_HAL::micros() - time_total;

    // Log rotation
    AP::logger().Write(
        "MLI1", "TimeUS,p,q,r,q0,q1,q2,q3",
        "Qfffffff",
        time,
        (double)rtU_.measure.omega_Kb[0], (double)rtU_.measure.omega_Kb[1], (double)rtU_.measure.omega_Kb[2],
        (double)rtU_.measure.q_bg[0], (double)rtU_.measure.q_bg[1], (double)rtU_.measure.q_bg[2], (double)rtU_.measure.q_bg[3]
        );

    // Log translation
    AP::logger().Write(
        "MLI2", "TimeUS,axg,ayg,azg,uKg,vKg,wKg,xg,yg,zg",
        "Qfffffffff",
        time,
        (double)rtU_.measure.a_Kg[0], (double)rtU_.measure.a_Kg[1], (double)rtU_.measure.a_Kg[2],
        (double)rtU_.measure.V_Kg[0], (double)rtU_.measure.V_Kg[1], (double)rtU_.measure.V_Kg[2],
        (double)rtU_.measure.s_Kg[0], (double)rtU_.measure.s_Kg[1], (double)rtU_.measure.s_Kg[2]
        );

    // Log the execution times    
    AP::logger().Write(
        "MLPM", "TimeUS,TimeTotalUS,TimeStepUS,TimeLogUS",
        "QQQQ",
        time, time_total, time_step, time_log );    

    // Log inertial sensor filter signals
    AP::logger().Write(
        "MLFI", "TimeUS,pr,qr,rr,pn,qn,rn,pf,qf,rf,axr,ayr,azr,axf,ayf,azf",
        "Qfffffffffffffff",
        time,
        (double)Omega_Kb_raw[0], (double)Omega_Kb_raw[1], (double)Omega_Kb_raw[2],
        (double)Omega_Kb_ntch[0], (double)Omega_Kb_ntch[1], (double)Omega_Kb_ntch[2],
        (double)Omega_Kb[0], (double)Omega_Kb[1], (double)Omega_Kb[2],
        (double)acc_FRD_raw[0], (double)acc_FRD_raw[1], (double)acc_FRD_raw[2],
        (double)acc_FRD[0], (double)acc_FRD[1], (double)acc_FRD[2]
        );

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

void ModeCustom::write_log_custom(const char *name, const char *labels, float *sf, int size, uint64_t time) {
    double s[size];
    for (int i=0; i<size; i++) {
        s[i] = (double)sf[i];
    }
    if (size==0) {
        return;
    } else if (size==1) {
        AP::logger().Write(name, labels,"Qf",time,s[0]);
    } else if (size==2) {
        AP::logger().Write(name, labels,"Qff",time,s[0],s[1]);
    } else if (size==3) {
        AP::logger().Write(name, labels,"Qfff",time,s[0],s[1],s[2]);
    } else if (size==4) {
        AP::logger().Write(name, labels,"Qffff",time,s[0],s[1],s[2],s[3]);
    } else if (size==5) {
        AP::logger().Write(name, labels,"Qfffff",time,s[0],s[1],s[2],s[3],s[4]);
    } else if (size==6) {
        AP::logger().Write(name, labels,"Qffffff",time,s[0],s[1],s[2],s[3],s[4],s[5]);
    } else if (size==7) {
        AP::logger().Write(name, labels,"Qfffffff",time,s[0],s[1],s[2],s[3],s[4],s[5],s[6]);
    } else if (size==8) {
        AP::logger().Write(name, labels,"Qffffffff",time,s[0],s[1],s[2],s[3],s[4],s[5],s[6],s[7]);
    } else if (size==9) {
        AP::logger().Write(name, labels,"Qfffffffff",time,s[0],s[1],s[2],s[3],s[4],s[5],s[6],s[7],s[8]);
    } else if (size==10) {
        AP::logger().Write(name, labels,"Qffffffffff",time,s[0],s[1],s[2],s[3],s[4],s[5],s[6],s[7],s[8],s[9]);
    } else if (size==11) {
        AP::logger().Write(name, labels,"Qfffffffffff",time,s[0],s[1],s[2],s[3],s[4],s[5],s[6],s[7],s[8],s[9],s[10]);
    } else if (size==12) {
        AP::logger().Write(name, labels,"Qffffffffffff",time,s[0],s[1],s[2],s[3],s[4],s[5],s[6],s[7],s[8],s[9],s[10],s[11]);
    } else if (size==13) {
        AP::logger().Write(name, labels,"Qfffffffffffff",time,s[0],s[1],s[2],s[3],s[4],s[5],s[6],s[7],s[8],s[9],s[10],s[11],s[12]);
    } else if (size==14) {
        AP::logger().Write(name, labels,"Qffffffffffffff",time,s[0],s[1],s[2],s[3],s[4],s[5],s[6],s[7],s[8],s[9],s[10],s[11],s[12],s[13]);
    } else if (size==15) {
        AP::logger().Write(name, labels,"Qfffffffffffffff",time,s[0],s[1],s[2],s[3],s[4],s[5],s[6],s[7],s[8],s[9],s[10],s[11],s[12],s[13],s[14]);
    }
};

void ModeCustom::extract_one_signal_name(const uint8_t log_names_int[], int number, signal_name_t &log_name){
    int idx = (number-1)*max_signal_name_length;
    for (int i=0;i<max_signal_name_length;i++) {
        log_name[i] = log_names_int[idx+i];
    }
};

Vector3f ModeCustom::imu_accel_to_cog_accel(Vector3f accel_ef_at_imu,Vector3f imu_offset,Vector3f Omega_Kb,Matrix3f M_gb){
    Vector3f accel_ef_at_cog;
    Vector3f Delta_accel_frd;
    Vector3f Delta_accel_ef;
    float p = Omega_Kb[0];
    float q = Omega_Kb[1];
    float r = Omega_Kb[2];
    float x = AP::ins().get_imu_pos_offset().x;
    float y = AP::ins().get_imu_pos_offset().y;
    float z = AP::ins().get_imu_pos_offset().z;
    // neglect (presumably noisy) angular acceleration influence - this could be tested and added in the future
    float dot_p = 0;
    float dot_q = 0;
    float dot_r = 0;
    // Brockhaus (2011). Flugregelung, Eq. (2.6.12)
    Delta_accel_frd[0] = dot_q*z-dot_r*y + q*p*y-q*q*x-r*r*x+r*p*z;
    Delta_accel_frd[1] = dot_r*x-dot_p*z + r*q*z-r*r*y-p*p*y+p*q*x;
    Delta_accel_frd[2] = dot_p*y-dot_q*x + p*r*x-p*p*z-q*q*z+q*r*y;
    // from forward-right-down frame (frd) to earth-fixed frame (ef)
    Delta_accel_ef = M_gb * Delta_accel_frd;
    accel_ef_at_cog = accel_ef_at_imu - Delta_accel_ef;
    return accel_ef_at_cog;
};
