#include "stm32f1xx_hal.h"

#include "cmd_listener.h"

#include "logging.h"

#define LOG_TAG ("CmdListener")

CmdListener::CmdListener()
{}

CmdListener& CmdListener::GetInstance()
{
    static CmdListener cmdListener;
    return cmdListener;
}

bool CmdListener::Init()
{
    Receiver& receiver = Receiver::GetInstance();
    if (!receiver.Init()) return false;

    return true;
}

bool CmdListener::Start()
{
    Receiver& receiver = Receiver::GetInstance();
    if (!receiver.Start()) {
        LOGI("Waiting for receiver to start\r\n");
        HAL_Delay(1000);
        while (!receiver.Start()) {
            HAL_Delay(1000);
        }
    }
    return true;
}

ReceiverStatus CmdListener::GetCmd(FCCmdType& cmd)
{
    return Receiver::GetInstance().GetCmd(cmd);
}