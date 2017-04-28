#ifndef TOPVIGRILLINTERFACE_H
#define TOPVIGRILLINTERFACE_H

#include "global.h"
#include <QList>

using namespace std;

class TopViCaptureInterface;

enum TopViCmd {
    TPV_INIT = 1,
    TPV_STATUS,
    TPV_GRAB,
    TPV_SET_EXPOSURE,
    TPV_SET_PREVIEW,
    TPV_FOCUS_MODE,
    TPV_SET_CONTROL,
    TPV_GET_CONTROL
};

struct grill_request {
    string requestStr;
};

enum TopViGrillResult {
    GRILL_ERROR,
    GRILL_OK
};

struct grill_reply {
    TopViGrillResult replyResult;
    int msgLength;
    int cmdId;
    string replyStr;
};

enum TopViCmdStatus{
    TPV_UNKNOWN = -1,
    TPV_OK,
    TPV_QUERY,
    TPV_NO_REPLY,
    TPV_WAIT_REPLY,
    TPV_REPLY
};

struct TopViGrillCommand {
    TopViCaptureInterface *parent;
    int cmdId;
    TopViCmd cmdType;
    int camId;
    TopViCmdStatus state;
    grill_request *request;
    grill_reply *reply;

};

string generateCommand(TopViCmd cmdType, int camId, int value, int additional_value = 0);
grill_reply *parseReply(string rstr);
string cmdTypeName(TopViCmd cmdType);
string cmdStatusName(TopViCmdStatus cmdStatus);
TopViCmdStatus cmdNeedReplay(TopViCmd cmdType);
string generateTestReply(TopViCmd cmdType);

class TopViGrillInterface
{
public:

    QList<TopViGrillCommand*> commandList;

    TopViGrillInterface(){    }

    int addCommand(TopViCaptureInterface *parent, TopViCmd cmdType, int camId, int value, int add_value);
    void removeOldCommands();
    void replyCallback(string reply);

private:
    int lastCmdId = 0;
};

#endif // TOPVIGRILLINTERFACE_H
