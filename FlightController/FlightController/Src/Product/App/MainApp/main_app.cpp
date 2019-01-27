#include "main_app.h"

#include "UAV_Defines.h"
#include "state_estimator.h"
#include "cmd_listener.h"
#include "controller.h"
#include "sensor_reader.h"

/*
 * Constants
 */

#define CORE_TIMER_FREQ 1000

#define READ_SENSOR_CNT 20 // 1000/50hz
#define ESTIMATE_STATE_CNT 20 // 1000/50hz
#define CONTROLL_CNT 20 // 1000/50hz
#define LISTEN_CMD_CNT 20 // 1000/50hz

/*
 *Static
 */

static int sTimerCnt = 0;

static StateEstimator& sStateEstimator = StateEstimator::GetInstance();
static Controller& sController = Controller::GetInstance();
static SensorReader& sSensorReader = SensorReader::GetInstance();
static CmdListener& sCmdListener = CmdListener::GetInstance();

static FCSensorMeasType sMeas;

void MainApp()
{
}

void OnCoreTimerTick()
{
    ++sTimerCnt;
    if (sTimerCnt == READ_SENSOR_CNT) {
        sSensorReader.GetSensorMeas(sMeas);
    }
    if (sTimerCnt == LISTEN_CMD_CNT) {
        FCCmdType cmd;
        sCmdListener.GetCmd(cmd);
        sController.SetAccSetpoint(cmd.desiredAcc);
        sController.SetYawSetpoint(cmd.desiredYaw);
    }
    if (sTimerCnt == ESTIMATE_STATE_CNT) {
        sStateEstimator.EstimateState(sMeas);
        sController.SetCurAtt(sStateEstimator.mState.att);
        sController.SetCurAttRate(sStateEstimator.mState.attRate);
    }
    if (sTimerCnt == CONTROLL_CNT) {
        sController.Run();
    }

    if (sTimerCnt >= 1000) {
        sTimerCnt = 0;
    }
}