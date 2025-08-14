#pragma once

#include <windows.h>
#include <inttypes.h>

#define TRIGGER_EFFECT_CONTROL_POINT_NUM 10

enum EResultType {
  Result_None = 0,
  Result_NotInitialized = 1,
  Result_AlreadyInitialized = 2,
};

enum EVRControllerType {
  VRController_Left = 0,
  VRController_Right = 1,
  VRController_Both = 2,
};

#ifdef __cplusplus
extern "C" {
#endif

  // Initialize the PS VR2 Toolkit CAPI library. You must call this function EXACTLY ONCE
  // at the beginning of your program.
  EResultType WINAPI psvr2_toolkit_init();

  EResultType WINAPI psvr2_toolkit_set_trigger_effect_off(EVRControllerType controllerType);
  EResultType WINAPI psvr2_toolkit_set_trigger_effect_feedback(EVRControllerType controllerType, uint8_t position, uint8_t strength);
  EResultType WINAPI psvr2_toolkit_set_trigger_effect_weapon(EVRControllerType controllerType, uint8_t startPosition, uint8_t endPosition, uint8_t strength);
  EResultType WINAPI psvr2_toolkit_set_trigger_effect_vibration(EVRControllerType controllerType, uint8_t position, uint8_t amplitude, uint8_t frequency);
  EResultType WINAPI psvr2_toolkit_set_trigger_effect_multiple_position_feedback(EVRControllerType controllerType, uint8_t strength[TRIGGER_EFFECT_CONTROL_POINT_NUM]);
  EResultType WINAPI psvr2_toolkit_set_trigger_effect_slope_feedback(EVRControllerType controllerType, uint8_t startPosition, uint8_t endPosition, uint8_t startStrength, uint8_t endStrength);
  EResultType WINAPI psvr2_toolkit_set_trigger_effect_multiple_position_vibration(EVRControllerType controllerType, uint8_t frequency, uint8_t amplitude[TRIGGER_EFFECT_CONTROL_POINT_NUM]);

  EResultType WINAPI psvr2_toolkit_get_gaze_state();

  /* Private functions */

  EResultType WINAPI psvr2_toolkit_private_set_gaze_enabled_eye();

  EResultType WINAPI psvr2_toolkit_private_start_gaze_calibration();
  EResultType WINAPI psvr2_toolkit_private_stop_gaze_calibration();

  EResultType WINAPI psvr2_toolkit_private_set_gaze_calibration_point();

  /* Unknown gaze functions */

  EResultType WINAPI psvr2_toolkit_private_gaze_0x08a964bcd9e0a1f4();
  EResultType WINAPI psvr2_toolkit_private_gaze_0x913203ac32f332fd();

#ifdef __cplusplus
}
#endif
