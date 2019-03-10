#include <string.h>
#include <stdio.h>

#include "sbus.h"
#include "logging.h"

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

    cmd.desiredAcc.z = sbusData.channels[0];
    cmd.desiredAcc.y = sbusData.channels[1];
    cmd.desiredAcc.x = sbusData.channels[2];
    cmd.desiredYaw = sbusData.channels[3];

    if (sbusData.failsafe) return RECEIVER_FAILSAFE;
    if (sbusData.lostFrame) return RECEIVER_LOST_FRAME;

    return RECEIVER_SUCCESS;
}

