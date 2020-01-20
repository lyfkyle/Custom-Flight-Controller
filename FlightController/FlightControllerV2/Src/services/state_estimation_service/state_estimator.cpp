#include "stm32f1xx_hal.h"

#include "logging.h"

#include "state_estimator.h"

/*
 * Defines
 */

#define LOG_TAG ("StateEstimator")

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

bool StateEstimator::Init()
{
    return true;
}

bool StateEstimator::EstimateState(FCSensorMeasType& meas)
{
    // All this flipping is because IMU is mounted upside down
    mFilter.updateIMU(-meas.gyroData.x, -meas.gyroData.y, -meas.gyroData.z,
                      meas.accData.x, meas.accData.y, -meas.accData.z);
    mState.att.roll = -mFilter.getRollRadians();
    mState.att.pitch = mFilter.getPitchRadians();
    mState.att.yaw = mFilter.getYawRadians();
    mState.attRate.roll = meas.gyroData.x;
    mState.attRate.pitch = -meas.gyroData.y;
    mState.attRate.yaw = -meas.gyroData.z;

    /*
    FCQuaternionType quat;
    mFilter.GetQuat(&quat);
    PRINT("Q: %f %f %f %f %u\r\n", quat.q1, quat.q2, quat.q3, quat.q4, HAL_GetTick()); // this will send through UART as well
    */
    return true;
}