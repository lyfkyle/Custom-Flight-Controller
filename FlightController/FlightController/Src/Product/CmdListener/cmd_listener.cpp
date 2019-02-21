#include <string.h>
#include <stdio.h>

#include "cmd_listener.h"

#include "sbus.h"
#include "logging.h"

#define LOG_TAG ("CmdListener")

CmdListener::CmdListener()
{

}

CmdListener& CmdListener::GetInstance()
{
    static CmdListener cmdListener;
    return cmdListener;
}

CmdListenerStatus CmdListener::GetCmd(FCCmdType& cmd)
{
    SBUSDataType sbusData;
    memset(&sbusData, 0, sizeof(SBUSDataType));
    if (!SBUS_Read(&sbusData)) {
        LOGE("Failed to read SBUS data\r\n");
        return CMD_LISTENER_FAIL;
    }

    cmd.desiredAcc.z = sbusData.channels[0];
    cmd.desiredAcc.y = sbusData.channels[1];
    cmd.desiredAcc.x = sbusData.channels[2];
    cmd.desiredYaw = sbusData.channels[3];

    if (sbusData.failsafe) return CMD_LISTENER_FAILSAFE;
    if (sbusData.lostFrame) return CMD_LISTENER_LOST_FRAME;

    return CMD_LISTENER_SUCCESS;
}