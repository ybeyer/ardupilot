#pragma once

// #define Custom_Matlab_Output //define for the custom simulink output
#define Custom_Debug // print performance to console
#define Mode_Custom_Use_External_Controller

#include <AP_Param/AP_Param.h>
#include <AP_Common/Location.h>
#include <stdint.h>
#include <AP_Soaring/AP_Soaring.h>
#include <AP_ADSB/AP_ADSB.h>
#include <AP_Vehicle/ModeReason.h>
#include "quadplane.h"
#include <AP_Common/MatlabController.h>
#ifdef Custom_Matlab_Output
    #include <AP_HAL/utility/Socket.h>
#endif

class AC_PosControl;
class AC_AttitudeControl_Multi;
class AC_Loiter;
class Mode
{
public:

    /* Do not allow copies */
    Mode(const Mode &other) = delete;
    Mode &operator=(const Mode&) = delete;

    // Auto Pilot modes
    // ----------------
    enum Number : uint8_t {
        MANUAL        = 0,
        CIRCLE        = 1,
        STABILIZE     = 2,
        TRAINING      = 3,
        ACRO          = 4,
        FLY_BY_WIRE_A = 5,
        FLY_BY_WIRE_B = 6,
        CRUISE        = 7,
        AUTOTUNE      = 8,
        AUTO          = 10,
        RTL           = 11,
        LOITER        = 12,
        TAKEOFF       = 13,
        AVOID_ADSB    = 14,
        GUIDED        = 15,
        INITIALISING  = 16,
#if HAL_QUADPLANE_ENABLED
        QSTABILIZE    = 17,
        QHOVER        = 18,
        QLOITER       = 19,
        QLAND         = 20,
        QRTL          = 21,
#if QAUTOTUNE_ENABLED
        QAUTOTUNE     = 22,
#endif
        QACRO         = 23,
#endif
        THERMAL       = 24,
#if HAL_QUADPLANE_ENABLED
        LOITER_ALT_QLAND = 25,
#endif
        CUSTOM        = 26, //add mode 
    };

    // Constructor
    Mode();

    // enter this mode, always returns true/success
    bool enter();

    // perform any cleanups required:
    void exit();

    // run controllers specific to this mode
    virtual void run() {};

    // returns a unique number specific to this mode
    virtual Number mode_number() const = 0;

    // returns full text name
    virtual const char *name() const = 0;

    // returns a string for this flightmode, exactly 4 bytes
    virtual const char *name4() const = 0;

    // returns true if the vehicle can be armed in this mode
    virtual bool allows_arming() const { return true; }

    //
    // methods that sub classes should override to affect movement of the vehicle in this mode
    //

    // convert user input to targets, implement high level control for this mode
    virtual void update() = 0;

    // true for all q modes
    virtual bool is_vtol_mode() const { return false; }
    virtual bool is_vtol_man_throttle() const;
    virtual bool is_vtol_man_mode() const { return false; }

    // guided or adsb mode
    virtual bool is_guided_mode() const { return false; }

    // true if mode can have terrain following disabled by switch
    virtual bool allows_terrain_disable() const { return false; }

    // true if automatic switch to thermal mode is supported.
    virtual bool does_automatic_thermal_switch() const {return false; }

    // subclasses override this if they require navigation.
    virtual void navigate() { return; }

    // this allows certain flight modes to mix RC input with throttle
    // depending on airspeed_nudge_cm
    virtual bool allows_throttle_nudging() const { return false; }

    // true if the mode sets the vehicle destination, which controls
    // whether control input is ignored with STICK_MIXING=0
    virtual bool does_auto_navigation() const { return false; }

    // true if the mode sets the vehicle destination, which controls
    // whether control input is ignored with STICK_MIXING=0
    virtual bool does_auto_throttle() const { return false; }

    // method for mode specific target altitude profiles
    virtual bool update_target_altitude() { return false; }

    // handle a guided target request from GCS
    virtual bool handle_guided_request(Location target_loc) { return false; }

protected:

    // subclasses override this to perform checks before entering the mode
    virtual bool _enter() { return true; }

    // subclasses override this to perform any required cleanup when exiting the mode
    virtual void _exit() { return; }

#if HAL_QUADPLANE_ENABLED
    // References for convenience, used by QModes
    AC_PosControl*& pos_control;
    AC_AttitudeControl_Multi*& attitude_control;
    AC_Loiter*& loiter_nav;
    QuadPlane& quadplane;
    QuadPlane::PosControlState &poscontrol;
#endif

    MatlabControllerClass custom_controller;
#ifdef Custom_Matlab_Output
    SocketAPM socket_debug; //
    const char *_debug_address = "127.0.0.1";
    int _debug_port = 9004;
#endif
};


class ModeAcro : public Mode
{
public:

    Mode::Number mode_number() const override { return Mode::Number::ACRO; }
    const char *name() const override { return "ACRO"; }
    const char *name4() const override { return "ACRO"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
};

class ModeAuto : public Mode
{
public:

    Number mode_number() const override { return Number::AUTO; }
    const char *name() const override { return "AUTO"; }
    const char *name4() const override { return "AUTO"; }

    bool does_automatic_thermal_switch() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override;

    bool does_auto_throttle() const override;

protected:

    bool _enter() override;
    void _exit() override;
};


class ModeAutoTune : public Mode
{
public:

    Number mode_number() const override { return Number::AUTOTUNE; }
    const char *name() const override { return "AUTOTUNE"; }
    const char *name4() const override { return "ATUN"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
    void _exit() override;
};

class ModeGuided : public Mode
{
public:

    Number mode_number() const override { return Number::GUIDED; }
    const char *name() const override { return "GUIDED"; }
    const char *name4() const override { return "GUID"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    virtual bool is_guided_mode() const override { return true; }

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

    // handle a guided target request from GCS
    bool handle_guided_request(Location target_loc) override;

protected:

    bool _enter() override;
};

class ModeCircle: public Mode
{
public:

    Number mode_number() const override { return Number::CIRCLE; }
    const char *name() const override { return "CIRCLE"; }
    const char *name4() const override { return "CIRC"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;
};

class ModeLoiter : public Mode
{
public:

    Number mode_number() const override { return Number::LOITER; }
    const char *name() const override { return "LOITER"; }
    const char *name4() const override { return "LOIT"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool isHeadingLinedUp(const Location loiterCenterLoc, const Location targetLoc);
    bool isHeadingLinedUp_cd(const int32_t bearing_cd);

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }
    
    bool allows_terrain_disable() const override { return true; }

protected:

    bool _enter() override;
};

#if HAL_QUADPLANE_ENABLED
class ModeLoiterAltQLand : public ModeLoiter
{
public:

    Number mode_number() const override { return Number::LOITER_ALT_QLAND; }
    const char *name() const override { return "Loiter to QLAND"; }
    const char *name4() const override { return "L2QL"; }

    // handle a guided target request from GCS
    bool handle_guided_request(Location target_loc) override;

protected:
    bool _enter() override;

    void navigate() override;

private:
    void switch_qland();

};
#endif // HAL_QUADPLANE_ENABLED

class ModeManual : public Mode
{
public:

    Number mode_number() const override { return Number::MANUAL; }
    const char *name() const override { return "MANUAL"; }
    const char *name4() const override { return "MANU"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
};

class ModeCustom : public Mode //added
{
public:

    Number mode_number() const override { return Number::CUSTOM; }
    const char* name() const override { return "CUSTOM"; }
    const char* name4() const override { return "CUSTOM"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
    void add_waypoint(uint16_T index, Vector3f location);
    void add_speed(uint16_T index, float V_k);
    void mission_updated(){updated_waypoints = true;};

protected:

    bool _enter() override;
    static const int max_num_of_matlab_waypoints = 10;
    // Ardupilot contains ghost waypoints
    // (home position and velocity of previous waypoint),
    // this is the max size
    static const int max_num_of_ardupilot_waypoints = 2*max_num_of_matlab_waypoints+1;
    int numberOfNavCommands = 0;
    float waypoints[max_num_of_ardupilot_waypoints][4];
    // will be set true in case of mission update through function mission_updated
    bool updated_waypoints = false;
 
    // custom logging
    static const int num_log_batches = sizeof(log_config)/sizeof(log_config[0]);
    static const int max_num_signals_per_batch = 14;
    static const int max_signal_name_length = 3;
    static const int max_batch_name_length = 4;
    typedef uint8_t signal_name_t[max_signal_name_length];
    char label_full[num_log_batches][6+max_num_signals_per_batch*(max_signal_name_length+1)+1];
    int label_length[num_log_batches];
    char batch_name_full[num_log_batches][max_batch_name_length];
    int batch_name_length[num_log_batches];
    int log_signal_idx_cumsum[num_log_batches];
    // log initialization function
    void log_setup(const logConfigBus log_config_in[]);
    // set log labels (e.g. "TimeUS,s1,s2,s3") that are passed to AP::logger().Write(…)
    void set_log_labels(const logConfigBus log_config[]);
    // set log batch names (e.g. "ML1" or "MLXY") that are passed to AP::logger().Write(…)
    void set_log_batch_names(const logConfigBus log_config[]);
    // set auxilliary cumulative index that is needed to pick the log signals from the log signals array
    void set_log_signal_idx_cumsum(const logConfigBus log_config[]);
    // wrapper of AP::logger().Write() for use of arrays (implementaion does not look good but there is probably no simpler alternative)
    void write_log_custom(const char *name, const char *labels, float *signals, int size);
    // signal names are part of the label (e.g. "s1" or "s2" or "s3")
    void extract_one_signal_name(const uint8_t log_names_int[], int number, signal_name_t &log_name);

    #ifdef Mode_Custom_Use_External_Controller
        void step_external();
    #endif
};

class ModeRTL : public Mode
{
public:

    Number mode_number() const override { return Number::RTL; }
    const char *name() const override { return "RTL"; }
    const char *name4() const override { return "RTL "; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;

private:

    // Switch to QRTL if enabled and within radius
    bool switch_QRTL(bool check_loiter_target = true);
};

class ModeStabilize : public Mode
{
public:

    Number mode_number() const override { return Number::STABILIZE; }
    const char *name() const override { return "STABILIZE"; }
    const char *name4() const override { return "STAB"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
};

class ModeTraining : public Mode
{
public:

    Number mode_number() const override { return Number::TRAINING; }
    const char *name() const override { return "TRAINING"; }
    const char *name4() const override { return "TRAN"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
};

class ModeInitializing : public Mode
{
public:

    Number mode_number() const override { return Number::INITIALISING; }
    const char *name() const override { return "INITIALISING"; }
    const char *name4() const override { return "INIT"; }

    bool _enter() override { return false; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    bool allows_arming() const override { return false; }

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_throttle() const override { return true; }
};

class ModeFBWA : public Mode
{
public:

    Number mode_number() const override { return Number::FLY_BY_WIRE_A; }
    const char *name() const override { return "FLY_BY_WIRE_A"; }
    const char *name4() const override { return "FBWA"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

};

class ModeFBWB : public Mode
{
public:

    Number mode_number() const override { return Number::FLY_BY_WIRE_B; }
    const char *name() const override { return "FLY_BY_WIRE_B"; }
    const char *name4() const override { return "FBWB"; }

    bool allows_terrain_disable() const override { return true; }

    bool does_automatic_thermal_switch() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;
};

class ModeCruise : public Mode
{
public:

    Number mode_number() const override { return Number::CRUISE; }
    const char *name() const override { return "CRUISE"; }
    const char *name4() const override { return "CRUS"; }

    bool allows_terrain_disable() const override { return true; }

    bool does_automatic_thermal_switch() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool get_target_heading_cd(int32_t &target_heading) const;

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;

    bool locked_heading;
    int32_t locked_heading_cd;
    uint32_t lock_timer_ms;
};

#if HAL_ADSB_ENABLED
class ModeAvoidADSB : public Mode
{
public:

    Number mode_number() const override { return Number::AVOID_ADSB; }
    const char *name() const override { return "AVOID_ADSB"; }
    const char *name4() const override { return "AVOI"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    virtual bool is_guided_mode() const override { return true; }

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;
};
#endif

#if HAL_QUADPLANE_ENABLED
class ModeQStabilize : public Mode
{
public:

    Number mode_number() const override { return Number::QSTABILIZE; }
    const char *name() const override { return "QSTABILIZE"; }
    const char *name4() const override { return "QSTB"; }

    bool is_vtol_mode() const override { return true; }
    bool is_vtol_man_throttle() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }
    bool allows_throttle_nudging() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // used as a base class for all Q modes
    bool _enter() override;

    void run() override;

protected:
private:

    void set_tailsitter_roll_pitch(const float roll_input, const float pitch_input);
    void set_limited_roll_pitch(const float roll_input, const float pitch_input);

};

class ModeQHover : public Mode
{
public:

    Number mode_number() const override { return Number::QHOVER; }
    const char *name() const override { return "QHOVER"; }
    const char *name4() const override { return "QHOV"; }

    bool is_vtol_mode() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

protected:

    bool _enter() override;
};

class ModeQLoiter : public Mode
{
friend class QuadPlane;
friend class ModeQLand;
public:

    Number mode_number() const override { return Number::QLOITER; }
    const char *name() const override { return "QLOITER"; }
    const char *name4() const override { return "QLOT"; }

    bool is_vtol_mode() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

protected:

    bool _enter() override;
};

class ModeQLand : public Mode
{
public:

    Number mode_number() const override { return Number::QLAND; }
    const char *name() const override { return "QLAND"; }
    const char *name4() const override { return "QLND"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

    bool allows_arming() const override { return false; }

protected:

    bool _enter() override;
};

class ModeQRTL : public Mode
{
public:

    Number mode_number() const override { return Number::QRTL; }
    const char *name() const override { return "QRTL"; }
    const char *name4() const override { return "QRTL"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

    bool allows_arming() const override { return false; }

    bool does_auto_throttle() const override { return true; }

    bool update_target_altitude() override;

    bool allows_throttle_nudging() const override;

protected:

    bool _enter() override;

private:

    enum class SubMode {
        climb,
        RTL,
    } submode;
};

class ModeQAcro : public Mode
{
public:

    Number mode_number() const override { return Number::QACRO; }
    const char *name() const override { return "QACO"; }
    const char *name4() const override { return "QACRO"; }

    bool is_vtol_mode() const override { return true; }
    bool is_vtol_man_throttle() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

protected:

    bool _enter() override;
};

#if QAUTOTUNE_ENABLED
class ModeQAutotune : public Mode
{
public:

    Number mode_number() const override { return Number::QAUTOTUNE; }
    const char *name() const override { return "QAUTOTUNE"; }
    const char *name4() const override { return "QATN"; }

    bool is_vtol_mode() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    void run() override;

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
    void _exit() override;
};
#endif  // QAUTOTUNE_ENABLED

#endif  // HAL_QUADPLANE_ENABLED

class ModeTakeoff: public Mode
{
public:
    ModeTakeoff();

    Number mode_number() const override { return Number::TAKEOFF; }
    const char *name() const override { return "TAKEOFF"; }
    const char *name4() const override { return "TKOF"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

    // var_info for holding parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:
    AP_Int16 target_alt;
    AP_Int16 target_dist;
    AP_Int16 level_alt;
    AP_Int8 level_pitch;

    bool takeoff_started;
    Location start_loc;

    bool _enter() override;
};

#if HAL_SOARING_ENABLED

class ModeThermal: public Mode
{
public:

    Number mode_number() const override { return Number::THERMAL; }
    const char *name() const override { return "THERMAL"; }
    const char *name4() const override { return "THML"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // Update thermal tracking and exiting logic.
    void update_soaring();

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    // true if we are in an auto-throttle mode, which means
    // we need to run the speed/height controller
    bool does_auto_throttle() const override { return true; }

protected:

    bool exit_heading_aligned() const;
    void restore_mode(const char *reason, ModeReason modereason);

    bool _enter() override;
};

#endif
