#include "tpv.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum TopViGain tpvSetColorGain(const char *value) {
#if 0
    return atoi(value);
#else
    if (strncmp(value, "global", strlen("global")) == 0) return TPV_GLOBAL;
    if (strncmp(value, "red", strlen("red")) == 0) return TPV_RED;
    if (strncmp(value, "blue", strlen("blue")) == 0) return TPV_BLUE;
    return TPV_GLOBAL;
#endif
}

const char* tpvGenerateTestReply(enum TopViCmdType cmdType, enum TopViCmdName cmdName) {
    (void)cmdType;
    switch (cmdName) {
        case TPV_INIT: return "RE00c{1,0:9:0x00}";
        case TPV_STATUS: return "RE01e{2,2:0:0:3840:2780:12:1:4:100}";
        case TPV_GRAB: return "RE024{3,0:tpv_20170506T194628_cam0_1.pgm}";
        case TPV_EXPOSURE: return "";
        case TPV_GAIN: return "";
        case TPV_LIVE: return "";
        case TPV_FOCUS: return "RE009{5,3:125}";
        case TPV_CONTROL: return "";
        case TPV_CLEAN: return "";
        default: return "";
    }
    return "UNKNOWN";
}

const char *tpvCmdStatus(enum TopViCmdStatus cmdStatus) {
   switch (cmdStatus) {
        case TPV_QUERY: return "TPV_QUERY";
        case TPV_WAIT_REPLY: return "TPV_WAIT_REPLY";
        case TPV_REPLY: return "TPV_REPLY";
        case TPV_OK: return "TPV_OK";
        case TPV_NO_REPLY: return "TPV_NO_REPLY";
        case TPV_UNKNOWN: return "TPV_UNKNOWN";
   }
   return "Unknown cmd status";
}

enum TopViCmdStatus tpvCmdNeedReplay(enum TopViCmdType cmdType) {
    switch (cmdType) {
        case TPV_GET:
            return TPV_WAIT_REPLY;
        case TPV_SET:
            //return TPV_NO_REPLY;
            return TPV_WAIT_REPLY;
        case TPV_UNDEFINED:
            return TPV_NO_REPLY;
    }
    return TPV_UNKNOWN;
}

const char *tpvGenerateCommandString(enum TopViCmdType cmdType, enum TopViCmdName cmdName, int camId, const char* value, const char* additional_value){
    char buf[1024];
    size_t pos;
    pos = snprintf(buf, sizeof(buf), "%s,", tpvCmdType(cmdType));
    pos += snprintf(buf + pos, sizeof(buf) - pos, "/par/dev/vi/");
    if (camId > 0){
        pos += snprintf(buf + pos, sizeof(buf) - pos, "cam/%d/", camId);
    }
    if (cmdName != TPV_UNINITED) {
        if (cmdType == TPV_SET && cmdName == TPV_GAIN) {
            pos += snprintf(buf + pos, sizeof(buf) - pos, "%s", tpvGetColorGain(tpvSetColorGain(value)));
        }
        else {
            pos += snprintf(buf + pos, sizeof(buf) - pos, "%s", tpvCmdName(cmdName));
        }
    }
    else if (value && strlen(value) > 0){
        pos += snprintf(buf + pos, sizeof(buf) - pos, value);
    }
    if (/*cmdType == TPV_SET &&*/ additional_value && strlen(additional_value) > 0) {
        pos += snprintf(buf + pos, sizeof(buf) - pos, ",%s", additional_value);
    }
    return strdup(buf);
}

void tpvFreeString(const char *string){
    free((void*)string);
}

void tpvParseGrillReply(struct grill_reply *reply, const char *rstr){
    char status[3];
    char buf[0xFFF];
    int errCode = 0;
    sscanf(rstr, "%2s%x{%d,%[^}]}", status, &reply->msgLength, &reply->cmdName, buf);
    reply->replyStr = strdup(buf);
    if (strcmp(status, "OK") == 0){
        reply->replyResult = GRILL_OK;
    }
    else {
        reply->replyResult = GRILL_ERROR;
    }
    switch (reply->cmdName) {
        case TPV_GRAB: {
            char *res = reply->replyStr;
            sscanf(res, "%d:%s", &errCode, buf);
            tpvFreeString(res);
            reply->replyStr = strdup(buf);
            break;
        }
        default:
            errCode = 0;
    }
    printf("tpvParseReply() %s --> [RE%d,%s]:%s\n", rstr, reply->msgLength, tpvCmdName(reply->cmdName), reply->replyStr);
}
