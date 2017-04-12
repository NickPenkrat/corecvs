#include "topViGrillInterface.h"
#include "topViCapture.h"

string cmdTypeName(TopViCmd cmdType){
    switch (cmdType) {
        case TPV_ERROR: return "TPV_ERROR";
        case TPV_INIT: return "TPV_INIT";
        case TPV_STATUS: return "TPV_STATUS";
        case TPV_GRAB: return "TPV_GRAB";
        case TPV_SET_EXPOSURE: return "TPV_EXPOSURE";
        case TPV_SET_PREVIEW: return "TPV_PREVIEW";
        case TPV_FOCUS_MODE: return "TPV_FOCUS";
        case TPV_GET_CONTROL: return "TPV_GET_CONTROL";
        case TPV_SET_CONTROL: return "TPV_SET_CONTROL";
    }
    return "UNKNOWN";
}

TopViCmdStatus cmdNeedReplay(TopViCmd cmdType){
    switch (cmdType) {
        case TPV_STATUS:
        case TPV_GET_CONTROL:
        case TPV_FOCUS_MODE:
        case TPV_GRAB:
            return TPV_WAIT_REPLY;
        case TPV_ERROR:
        case TPV_INIT:
        case TPV_SET_EXPOSURE:
        case TPV_SET_PREVIEW:
        case TPV_SET_CONTROL:
            return TPV_NO_REPLY;
    }
    return TPV_UNKNOWN;
}

string generateTestReply(TopViCmd cmdType) {
    switch (cmdType) {
        case TPV_ERROR: return "ER";
        case TPV_INIT: return "";
        case TPV_STATUS: return "RE004<SU>";
        case TPV_GRAB:
            return "RE023ftp://speedtest.tele2.net/1KB.zip";
            //return "RE037ftp://193.232.110.156/tpv_20000101T010746_cam0_0.pgm";
        case TPV_SET_EXPOSURE: return "";
        case TPV_SET_PREVIEW: return "";
        case TPV_FOCUS_MODE: return "RE005{124}";
        case TPV_GET_CONTROL: return "RE004{}";
        case TPV_SET_CONTROL: return "";
    }
    return "TPV_UNKNOWN";
}

string generateCommandString(TopViCmd cmdType, int camId, int value, int additional_value){
    switch (cmdType) {
        case TPV_INIT:
            return "set,/par/dev/vi/restart,0";
        case TPV_STATUS:
            if (camId == 0) {
                return "print,/par/dev/vi/status";
            }
            else {
                return "print,/par/dev/vi/cam/" + to_string(camId) + "/status";
            }
        case TPV_GRAB:
            if (camId == 0) {
                return "print,/par/dev/vi/grab";
            }
            else {
                return "print,/par/dev/vi/cam/" + to_string(camId) + "/grab";
            }
        case TPV_SET_EXPOSURE:
            if (camId == 0) {
                return "set,/par/dev/vi/exposure," + to_string(value);
            }
            else {
                return "set,/par/dev/vi/cam/" + to_string(camId) + "/exposure," + to_string(value);
            }
        case TPV_SET_PREVIEW:
            if (camId == 0) {
                return "set,/par/dev/vi/live/," + to_string(value);
            }
            else {
                return "set,/par/dev/vi/cam/" + to_string(camId) + "/live," + to_string(value);
            }
        case TPV_FOCUS_MODE:
            if (camId == 0) {
                return "print,/par/dev/vi/focus";
            }
            else {
                return "print,/par/dev/vi/cam/" + to_string(camId) + "/focus";
            }
        case TPV_GET_CONTROL:
            if (camId == 0) {
                return "";
            }
            else {
                return "print,/par/dev/vi/cam/" + to_string(camId) + "/" + to_string(value);
            }
        case TPV_SET_CONTROL:
            if (camId == 0) {
                return "";
            }
            else {
                return "set,/par/dev/vi/cam/" + to_string(camId) + "/" + to_string(value) + "," + to_string(additional_value);
            }
        default:
            SYNC_PRINT(("COMMAND_INTERFACE: unknown command\n"));
            return "";
    }
    return "";
}

grill_reply *parseReply(string rstr){
    SYNC_PRINT(("parseReply(): %s\n", rstr.c_str()));
    grill_reply *reply = new grill_reply;
    reply->cmdId = 0;
    if (rstr.substr(0,2) == "ER"){
        reply->replyResult = GRILL_ERROR;
    }
    else {
        reply->replyResult = GRILL_OK;
        reply->msgLength = stoi(rstr.substr(2, 3));
        reply->replyStr = rstr.substr(5, rstr.length() - 5);
    }
    if (reply->replyResult == GRILL_OK) {
        SYNC_PRINT(("parseReply() %s --> [RE%d]:%s\n", rstr.c_str(), reply->msgLength, reply->replyStr.c_str()));
    }
    else {
        SYNC_PRINT(("parseReply() %s --> [ER]\n", rstr.c_str()));
    }
    return reply;
}

int TopViGrillInterface::addCommand(TopViCaptureInterface *parent, TopViCmd cmdType, int camId, int value, int add_value){
    TopViGrillCommand *cmd = new TopViGrillCommand;
    cmd->parent = parent;
    cmd->camId = camId;
    cmd->cmdType = cmdType;
    cmd->cmdId = lastCmdId;
    lastCmdId++;
    cmd->request = new grill_request;
    cmd->request->requestStr = generateCommandString(cmdType, camId, value, add_value);
    SYNC_PRINT(("TopViGrillInterface::addCommand(): command %s generated\n", cmd->request->requestStr.c_str()));
    cmd->reply = NULL;
    cmd->state = TPV_UNKNOWN;
    this->commandList.push_back(cmd);
    return 0;
}

void TopViGrillInterface::removeOldCommands(){
    QMutableListIterator<TopViGrillCommand*> it(commandList);
    while (it.hasNext()) {
         TopViGrillCommand *cmd = it.next();
         SYNC_PRINT(("TopViGrillInterface::removeCommand: cmdId = %d, cmdType = %s\n", cmd->cmdId, cmdTypeName(cmd->cmdType).c_str()));
         if (cmd->state == TPV_OK || cmd->state == TPV_NO_REPLY) {
             SYNC_PRINT(("Remove command %d\n", cmd->cmdId));
             it.remove();
             delete cmd->request;
             delete cmd->reply;
             delete cmd;
         }
    }
}

void TopViGrillInterface::replyCallback(string reply){
    SYNC_PRINT(("TopViGrillInterface::replyCallback() for %s\n", reply.c_str()));
    grill_reply *greply = parseReply(reply);
    QMutableListIterator<TopViGrillCommand*> it(commandList);
    while (it.hasNext()) {
         TopViGrillCommand *cmd = it.next();
         SYNC_PRINT(("TopViGrillInterface::replyCallback: cmdId = %d, cmdType = %s\n", cmd->cmdId, cmdTypeName(cmd->cmdType).c_str()));
         if (cmd->state == TPV_WAIT_REPLY /*&& greply->cmdId == cmd->cmdId*/) {
            cmd->reply = greply;
            cmd->state = TPV_REPLY;
         }
    }
}

