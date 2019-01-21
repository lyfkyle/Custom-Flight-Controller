#ifndef _CMD_LISTENER_H_
#define _CMD_LISTENER_H_

#include "UAV_Defines.h"

class CmdListener {
private:
    CmdListener(); // private constructor, singleton

    FCCmdType mCmd;
public:
    static CmdListener& GetInstance();

    bool GetCmd(FCCmdType& cmd);
};

#endif