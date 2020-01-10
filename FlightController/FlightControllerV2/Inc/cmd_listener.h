#ifndef _CMD_LISTENER_H_
#define _CMD_LISTENER_H_

#include "UAV_Defines.h"
#include "receiver.h"

class CmdListener {
private:
    CmdListener(); // private constructor, singleton

    FCCmdType mCmd;
public:
    static CmdListener& GetInstance();
    bool Init();
    bool Start();
    ReceiverStatus GetCmd(FCCmdType& cmd);
};

#endif