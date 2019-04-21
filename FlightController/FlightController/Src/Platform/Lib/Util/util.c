#include "util.h"

float Util_Constrain(float inVal, float inMin, float inMax, float outMin, float outMax)
{
    return (inVal - inMin) / (inMax - inMin) * (outMax - outMin) + outMin;
}

int Util_Constrain(int inVal, int inMin, int inMax, int outMin, int outMax)
{
    return (inVal - inMin) / (inMax - inMin) * (outMax - outMin) + outMin;
}