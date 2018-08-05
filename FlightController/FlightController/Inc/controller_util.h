#ifndef _CONTROLLER_UTIL_H_
#define _CONTROLLER_UTIL_H_

#include <UAV_Defines.h>

bool Controller_Init();

bool Controller_GetAttSetpointFromAccSetpoint(FCAttType* pAtt, double xAccSetpoint, double yAccSetpoint, double zAccSetpoint, double yawSetpoint);

#endif