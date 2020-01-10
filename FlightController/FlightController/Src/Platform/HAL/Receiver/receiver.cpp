#include <string.h>
#include <stdio.h>

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

    cmd.desiredVel.z = Util_Constrain((float)sbusData.channels[0], (float)SBUS_CHANNEL_MIN, (float)SBUS_CHANNEL_MAX, CMD_VEL_MIN, CMD_VEL_MAX);
    cmd.desiredVel.y = Util_Constrain((float)sbusData.channels[1], (float)SBUS_CHANNEL_MIN, (float)SBUS_CHANNEL_MAX, CMD_VEL_MIN, CMD_VEL_MAX);
    cmd.desiredVel.x = Util_Constrain((float)sbusData.channels[2], (float)SBUS_CHANNEL_MIN, (float)SBUS_CHANNEL_MAX, CMD_VEL_MIN, CMD_VEL_MAX);
    cmd.desiredYawRate = Util_Constrain((float)sbusData.channels[3], (float)SBUS_CHANNEL_MIN, (float)SBUS_CHANNEL_MAX, CMD_ACC_MIN, CMD_ACC_MAX);

    if (sbusData.failsafe) return RECEIVER_FAILSAFE;
    if (sbusData.lostFrame) return RECEIVER_LOST_FRAME;

    return RECEIVER_SUCCESS;
}

