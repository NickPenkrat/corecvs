#ifndef TOPVIGRILLINTERFACE_H
#define TOPVIGRILLINTERFACE_H

#include "global.h"
#include <QList>

#include "tpv.h"

using namespace std;

class TopViCaptureInterface;

struct TopViGrillCommand {
    TopViCaptureInterface *parent;
    int cmdId;
    enum TopViCmdName cmdName;
    enum TopViCmdType cmdType;
    int camId;
    enum TopViCmdStatus state;
    struct grill_request *request;
    struct grill_reply *reply;

};

class TopViGrillInterface
{
public:

    QList<TopViGrillCommand*> commandList;

    TopViGrillInterface(){    }

    int addCommand(TopViCaptureInterface *parent, TopViCmdType cmdType, TopViCmdName cmdName, int camId, string value, string add_value);
    void removeOldCommands();
    void addReplyToCommand(string reply);

private:
    int lastCmdId = 0;
};

#endif // TOPVIGRILLINTERFACE_H
