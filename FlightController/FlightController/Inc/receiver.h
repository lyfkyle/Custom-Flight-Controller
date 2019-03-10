#ifndef _RECEIVE_H_
#define _RECEIVE_H_

#include "UAV_Defines.h"

typedef enum {
    RECEIVER_SUCCESS,
    RECEIVER_FAIL,
    RECEIVER_FAILSAFE,
    RECEIVER_LOST_FRAME
} ReceiverStatus;

class Receiver {
private:
    // private constructor, singleton paradigm
    Receiver();
public:
    static Receiver& GetInstance();

    bool Init();
    bool Start();
    // bool Stop();
    ReceiverStatus GetCmd(FCCmdType& cmd);
};

#endif