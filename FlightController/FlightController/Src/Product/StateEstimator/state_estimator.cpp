#include "state_estimator.h"

/*
 * Defines
 */

#define DEFAULT_FILTER_FREQ 50 //hz

StateEstimator::StateEstimator() :
    mFilter()
{
    mState.att.roll = 0.0f;
    mState.att.yaw = 0.0f;
    mState.att.pitch = 0.0f;

    mFilter.begin(DEFAULT_FILTER_FREQ);
}

StateEstimator& StateEstimator::GetInstance()
{
    static StateEstimator stateEstimator;
    return stateEstimator;
}

bool StateEstimator::EstimateState(FCMeasType& meas)
{
    mFilter.update(meas.gyroData.x, meas.gyroData.y, meas.gyroData.z,
                   meas.accData.x, meas.accData.y, meas.accData.z,
                   meas.magData.x, meas.magData.y, meas.magData.z);
    mState.att.roll = mFilter.getRollRadians();
    mState.att.pitch = mFilter.getPitchRadians();
    mState.att.yaw = mFilter.getYawRadians();

    return true;
}