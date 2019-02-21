#ifndef _CMD_LISTENER_H_
#define _CMD_LISTENER_H_

#include "UAV_Defines.h"

typedef enum {
    CMD_LISTENER_SUCCESS,
    CMD_LISTENER_FAIL,
    CMD_LISTENER_FAILSAFE,
    CMD_LISTENER_LOST_FRAME
} CmdListenerStatus;

class CmdListener {
private:
    CmdListener(); // private constructor, singleton

    FCCmdType mCmd;
public:
    static CmdListener& GetInstance();

    CmdListenerStatus GetCmd(FCCmdType& cmd);
};

#endif