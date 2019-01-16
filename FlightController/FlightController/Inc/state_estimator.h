#ifndef _STATE_ESTIMATOR_H_
#define _STATE_ESTIMATOR_H_

#include "UAV_Defines.h"

#include "MadgwickAHRS.h"

typedef struct {
    FCAttType att;
} FCStateType;

typedef struct {
    FCSensorDataType gyroData;
    FCSensorDataType accData;
    FCSensorDataType magData;
} FCMeasType;

class StateEstimator {
private:
    Madgwick mFilter;

    StateEstimator(); // private constructor, singleton
public:
    FCStateType mState;

    static StateEstimator& GetInstance();
    bool EstimateState(FCMeasType& meas);
};

#endif