#include <string.h>
#include <stdio.h>
#include <math.h>

#include "sbus.h"
#include "logging.h"
#include "util.h"
#include "UAV_Defines.h"

#include "Receiver.h"

/*
 * Defines
 */

#define LOG_TAG ("Receiver")

/*
 * Code
 */

Receiver::Receiver()
{}

Receiver& Receiver::GetInstance()
{
    static Receiver receiver;
    return receiver;
}

bool Receiver::Init()
{
    return SBUS_Init();
}

bool Receiver::Start()
{
    return SBUS_Start();
}

ReceiverStatus Receiver::GetCmd(FCCmdType& cmd)
{
    SBUSDataType sbusData;
    memset(&sbusData, 0, sizeof(SBUSDataType));
    if (!SBUS_Read(&sbusData)) {
        LOGE("Failed to read SBUS data\r\n");
        return RECEIVER_FAIL;
    }

#if UAV_CMD_ACC
    cmd.desiredAcc.z = Util_Constrain((float)sbusData.channels[0], (float)SBUS_CHANNEL_MIN, (float)SBUS_CHANNEL_MAX, CMD_ACC_MIN, CMD_ACC_MAX);
    cmd.desiredAcc.y = Util_Constrain((float)sbusData.channels[1], (float)SBUS_CHANNEL_MIN, (float)SBUS_CHANNEL_MAX, CMD_ACC_MIN, CMD_ACC_MAX);
    cmd.desiredAcc.x = Util_Constrain((float)sbusData.channels[2], (float)SBUS_CHANNEL_MIN, (float)SBUS_CHANNEL_MAX, CMD_ACC_MIN, CMD_ACC_MAX);
    cmd.desiredYawRate = Util_Constrain((float)sbusData.channels[3], (float)SBUS_CHANNEL_MIN, (float)SBUS_CHANNEL_MAX, (float)CMD_YAW_RATE_MIN, (float)CMD_YAW_RATE_MAX);

    if (fabs(cmd.desiredVel.z) < 0.006) cmd.desiredVel.z = 0.0f;
    if (fabs(cmd.desiredVel.y) < 0.006) cmd.desiredVel.y = 0.0f;
    if (fabs(cmd.desiredVel.x) < 0.006) cmd.desiredVel.x = 0.0f;
    if (fabs(cmd.desiredYawRate) < 5) cmd.desiredYawRate = 0.0f;

#elif UAV_CMD_ATT
    cmd.desiredAccZ = Util_Constrain((float)sbusData.channels[0], (float)SBUS_CHANNEL_MIN, (float)SBUS_CHANNEL_MAX, CMD_ACC_MIN, CMD_ACC_MAX);
    cmd.desiredRoll = Util_Constrain((float)sbusData.channels[1], (float)SBUS_CHANNEL_MIN, (float)SBUS_CHANNEL_MAX, CMD_ROLL_MIN, CMD_ROLL_MAX);
    cmd.desiredPitch = Util_Constrain((float)sbusData.channels[2], (float)SBUS_CHANNEL_MIN, (float)SBUS_CHANNEL_MAX, CMD_PITCH_MIN, CMD_PITCH_MAX);
    cmd.desiredYawRate = Util_Constrain((float)sbusData.channels[3], (float)SBUS_CHANNEL_MIN, (float)SBUS_CHANNEL_MAX, (float)CMD_YAW_RATE_MIN, (float)CMD_YAW_RATE_MAX);

    if (fabs(cmd.desiredAccZ) < 0.006) cmd.desiredAccZ = 0.0f;
    if (fabs(cmd.desiredRoll) < 0.2) cmd.desiredRoll = 0.0f;
    if (fabs(cmd.desiredPitch) < 0.2) cmd.desiredPitch = 0.0f;
    if (fabs(cmd.desiredYawRate) < 5) cmd.desiredYawRate = 0.0f;
#endif

    if (sbusData.failsafe) return RECEIVER_FAILSAFE;
    if (sbusData.lostFrame) return RECEIVER_LOST_FRAME;

    return RECEIVER_SUCCESS;
}

