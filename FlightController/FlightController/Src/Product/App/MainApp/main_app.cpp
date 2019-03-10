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
 * Constants
 */

#define CORE_TIMER_FREQ 1000

#define READ_SENSOR_CNT 20 // 1000/50hz
#define ESTIMATE_STATE_CNT 20 // 1000/50hz
#define CONTROLL_CNT 20 // 1000/50hz
#define LISTEN_CMD_CNT 1 // 1000/1000hz

/*
 *Static
 */

static int sTimerCnt = 0;
static FCSensorMeasType sMeas;

/*
 * Code
 */

static void OnCoreTimerTick(void)
{
    ++sTimerCnt;
    if (sTimerCnt == READ_SENSOR_CNT) {
        SensorReader::GetInstance().GetSensorMeas(sMeas);
    }
    if (sTimerCnt == LISTEN_CMD_CNT) {
        FCCmdType cmd;
        ReceiverStatus status = CmdListener::GetInstance().GetCmd(cmd);
        if (status != RECEIVER_FAIL) {
            Controller::GetInstance().SetAccSetpoint(cmd.desiredAcc);
            Controller::GetInstance().SetYawSetpoint(cmd.desiredYaw);
        } else {
            LOGE("sCmdListener.GetCmd returns fail, skip\r\n");
        }
    }
    if (sTimerCnt == ESTIMATE_STATE_CNT) {
        StateEstimator::GetInstance().EstimateState(sMeas);
        Controller::GetInstance().SetCurAtt(StateEstimator::GetInstance().mState.att);
        Controller::GetInstance().SetCurAttRate(StateEstimator::GetInstance().mState.attRate);
    }
    if (sTimerCnt == CONTROLL_CNT) {
        Controller::GetInstance().Run();
    }

    if (sTimerCnt >= 1000) {
        sTimerCnt = 0;
    }
}

void MainApp()
{
    // everything ready. Let's go.
    // InterruptMgr_RegisterSystickHandler(OnCoreTimerTick);
}

