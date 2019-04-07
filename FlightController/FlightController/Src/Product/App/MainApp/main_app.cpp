#include "main_app.h"

#include "logging.h"
#include "UAV_Defines.h"
#include "interrupt_mgr.h"
#include "state_estimator.h"
#include "cmd_listener.h"
#include "controller.h"
#include "sensor_reader.h"

#define LOG_TAG ("MainApp")

/*
 * Defines
 */

#define MAIN_APP_DEBUG (0)
#if MAIN_APP_DEBUG
#define LOG(...) LOGI(__VA_ARGS__)
#else
#define LOG(...)
#endif

// test result, read data 13ms
//              estimate state 13ms
//

// TODO modulus
/*
 * Constants
 */

#define CORE_TIMER_FREQ 1000
#define TIMER_CNT_MAX 5000

#define READ_SENSOR_CNT 20 // 1000/50hz
#define ESTIMATE_STATE_CNT 20 // 1000/50hz
#define CONTROLL_CNT 1000 // 1000/50hz
#define LISTEN_CMD_CNT 1000 // 1000/1hz

/*
 *Static
 */

static int sTimerCnt = 0;
static int sReadSensorCnt = READ_SENSOR_CNT;
static int sEstimateStateCnt = ESTIMATE_STATE_CNT;
static int sControllerCnt = CONTROLL_CNT;
static int sListenCmdCnt = LISTEN_CMD_CNT;
static FCSensorMeasType sMeas;

/*
 * Code
 */

static void OnCoreTimerTick(void)
{
    ++sTimerCnt;
    if (sTimerCnt >= TIMER_CNT_MAX) {
        sTimerCnt = 0;
        sReadSensorCnt = READ_SENSOR_CNT;
        sEstimateStateCnt = ESTIMATE_STATE_CNT;
        sControllerCnt = CONTROLL_CNT;
        sListenCmdCnt = LISTEN_CMD_CNT;
    }
}

void MainApp()
{
    // everything ready. Let's go.
    LOGI("MainApp starts\r\n");
    InterruptMgr_RegisterSystickHandler(OnCoreTimerTick);
    while (1) {
        if (sTimerCnt >= sReadSensorCnt) {
            LOGI("readsensor : sTimerCnt = %d\r\n", sTimerCnt);
            SensorReader::GetInstance().GetSensorMeas(sMeas);
            LOGI("readsensor: sTimerCnt = %d\r\n", sTimerCnt);
            LOG("sensor meas: gyro: %f %f %f, acc: %f %f %f\r\n", sMeas.gyroData.x, sMeas.gyroData.y, sMeas.gyroData.z, sMeas.accData.x, sMeas.accData.y, sMeas.accData.z);
            sReadSensorCnt += READ_SENSOR_CNT;
        }
        if (sTimerCnt >= sListenCmdCnt) {
            LOGI("listencmd: sTimerCnt = %d\r\n", sTimerCnt);
            FCCmdType cmd;
            ReceiverStatus status = CmdListener::GetInstance().GetCmd(cmd);
            if (status != RECEIVER_FAIL) {
//                LOGI("Cmd: acc.x %f acc.y %f acc.z %f, yaw %f\r\n", cmd.desiredAcc.x, cmd.desiredAcc.y, cmd.desiredAcc.z, cmd.desiredYaw);
                Controller::GetInstance().SetAccSetpoint(cmd.desiredAcc);
                Controller::GetInstance().SetYawSetpoint(cmd.desiredYaw);
            } else {
                LOGE("sCmdListener.GetCmd returns fail, skip\r\n");
            }
            sListenCmdCnt += LISTEN_CMD_CNT;
            LOGI("listencmd: sTimerCnt = %d\r\n", sTimerCnt);
        }
        if (sTimerCnt >= sEstimateStateCnt) {
            LOGI("estimateState: sTimerCnt = %d\r\n", sTimerCnt);
            StateEstimator::GetInstance().EstimateState(sMeas);
//            LOGI("Estimated State: roll %f, pitch %f, yaw %f, rollRate %f, pitchRate %f, yawRate %f\r\n", StateEstimator::GetInstance().mState.att.roll, StateEstimator::GetInstance().mState.att.pitch,
//                 StateEstimator::GetInstance().mState.att.yaw, StateEstimator::GetInstance().mState.attRate.roll, StateEstimator::GetInstance().mState.attRate.pitch, StateEstimator::GetInstance().mState.attRate.yaw);
            Controller::GetInstance().SetCurAtt(StateEstimator::GetInstance().mState.att);
            Controller::GetInstance().SetCurAttRate(StateEstimator::GetInstance().mState.attRate);
            LOGI("estimateState: sTimerCnt = %d\r\n", sTimerCnt);
            sEstimateStateCnt += ESTIMATE_STATE_CNT;
        }
        if (sTimerCnt >= sControllerCnt) {
//            Controller::GetInstance().Run();
//            sControllerCnt += CONTROLL_CNT;
        }


    }
}

