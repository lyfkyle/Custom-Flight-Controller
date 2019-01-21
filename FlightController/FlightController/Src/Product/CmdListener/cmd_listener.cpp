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

bool CmdListener::GetCmd(FCCmdType& cmd)
{
    SBUSDataType sbusData;
    memset(&sbusData, 0, sizeof(SBUSDataType));
    if (!SBUS_Read(&sbusData)) {
        LOGE("Failed to read SBUS data\r\n");
        return false;
    }

    // TODO channel mapping
    mCmd.desiredAcc.x = sbusData.channels[0];
    mCmd.desiredAcc.y = sbusData.channels[1];
    mCmd.desiredAcc.z = sbusData.channels[2];
    mCmd.desiredYaw = sbusData.channels[3];

    return true;
}