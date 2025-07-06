#include "../../ArduCopter/Copter.h"
// - libraries/AP_Param/AP_Param.h
// - ArduCopter/mode.h
// - libraries/AC_AttitudeControl/MatlabController.h


const AP_Param::GroupInfo ModeCustom::var_info[] = {
  AP_SUBGROUPINFO(custom_controller, "", 0, ModeCustom, MatlabControllerClass),
	AP_GROUPEND
};
