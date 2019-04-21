#ifndef _CONTROLLER_UTIL_H_
#define _CONTROLLER_UTIL_H_

#include <UAV_Defines.h>

bool ControllerUtil_Init();

bool Controller_GetAttSetpointFromAccSetpoint(FCAttType& pAtt, FCAccDataType& pAccSetpoint, float yawSetpoint);
float GetHeightThrustFromVelSetpointZ(float velSetpoint_z);

#endif