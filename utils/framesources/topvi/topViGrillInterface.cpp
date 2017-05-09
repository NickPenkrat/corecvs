#include "topViGrillInterface.h"
#include "topViCapture.h"

int TopViGrillInterface::addCommand(TopViCaptureInterface *parent, TopViCmdType cmdType, TopViCmdName cmdName, int camId, string value, string add_value){
    TopViGrillCommand *cmd = new TopViGrillCommand;
    cmd->parent = parent;
    cmd->camId = camId;
    cmd->cmdType = cmdType;
    cmd->cmdName = cmdName;
    cmd->cmdId = lastCmdId;
    lastCmdId++;
    cmd->request = new grill_request;
    cmd->request->requestStr = tpvGenerateCommandString(cmdType, cmdName, camId, value.c_str(), add_value.c_str());
    SYNC_PRINT(("TopViGrillInterface::addCommand(): command %s generated\n", cmd->request->requestStr));
    cmd->reply = NULL;
    cmd->state = TPV_UNKNOWN;
    this->commandList.push_back(cmd);
    return 0;
}

void TopViGrillInterface::removeOldCommands(){
    QMutableListIterator<TopViGrillCommand*> it(commandList);
    while (it.hasNext()) {
         TopViGrillCommand *cmd = it.next();
         SYNC_PRINT(("TopViGrillInterface::removeCommand: cmdId = %d, cmdName = %s, cmdStatus = %s\n", cmd->cmdId, tpvCmdName(cmd->cmdName), tpvCmdStatus(cmd->state)));
         if (cmd->state == TPV_OK || cmd->state == TPV_NO_REPLY) {
             SYNC_PRINT(("Remove command %d with state %s\n", cmd->cmdId, tpvCmdStatus(cmd->state)));
             it.remove();
             tpvFreeString(cmd->request->requestStr);
             tpvFreeString(cmd->reply->replyStr);
             delete cmd->request;
             delete cmd->reply;
             delete cmd;
         }
    }
}

void TopViGrillInterface::addReplyToCommand(string reply){
    SYNC_PRINT(("TopViGrillInterface::replyCallback() for %s\n", reply.c_str()));
    grill_reply *greply = new grill_reply;
    tpvParseGrillReply(greply, reply.c_str());
    QMutableListIterator<TopViGrillCommand*> it(commandList);
    while (it.hasNext()) {
         TopViGrillCommand *cmd = it.next();
         SYNC_PRINT(("TopViGrillInterface::replyCallback: cmdId = %d, cmdName = %s\n", cmd->cmdId, tpvCmdName(cmd->cmdName)));
         if (cmd->state == TPV_WAIT_REPLY /*&& greply->cmdId == cmd->cmdId*/) {  
            cmd->reply = greply;
         }
    }
}

