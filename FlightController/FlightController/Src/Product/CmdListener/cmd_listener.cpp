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
    if (!receiver.Start()) return false;

    return true;
}

ReceiverStatus CmdListener::GetCmd(FCCmdType& cmd)
{
    return Receiver::GetInstance().GetCmd(cmd);
}