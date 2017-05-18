#include "tpv.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define GAIN_1 0x1040
#define GAIN_2 0x1840
#define GAIN_4 0x1C40
#define GAIN_8 0x1CC0
#define GAIN_16 0x1DC0

#define GAIN_INTERVAL 0x40

int tpvGetGainCode(double g) {
    int code, rem;
    if (g < 1 || g > 32)
        return -1;
    code = GAIN_1;
    if (g >= 2 && g < 4)  { g /= 2;  code = GAIN_2; }
    if (g >= 4 && g < 8)  { g /= 4;  code = GAIN_4; }
    if (g >= 8 && g < 16) { g /= 8;  code = GAIN_8; }
    if (g >= 16 && g < 32){ g /= 16; code = GAIN_16; }
    rem = (int)(g * GAIN_INTERVAL);
    code |= rem;
    return code;
}

double tpvGetGainValue(int code) {
    int gains[5]={GAIN_1, GAIN_2, GAIN_4, GAIN_8, GAIN_16};
    int i, j;
    double g = 1, rem = 0, delta = 0;
    //printf("code = 0x%x\n", code);
    for (i = 0; i < 5; i++) {
        if (code >= gains[i] && code < gains[i] + GAIN_INTERVAL) {
            g = 1;
            for (j = 0; j < i; j++) { g *= 2; }
            delta = (g * 2 - g) / (double)GAIN_INTERVAL;
            rem = (code - gains[i]) * delta;
            //printf("res = %d g = %lf, delta = %lf, rem = %lf\n", code-gains[i], g, delta, rem);
            g += rem;
            return g;
        }
    }
    return -1;
}

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

int tpvInitSK(struct TopVi_SK *sk) {
    sk->status = TPV_SK_UNKNOWN;
    sk->camId = -1;
    sk->param.activeMode = TPV_PK_UNKNOWN;
    sk->param.binningMode = TPV_PK_1X1;
    sk->param.size = TPV_PK_3840x2780;
    sk->param.exposure = 100.;
    sk->param.globalGain = 1;
    sk->param.blueGain = 1;
    sk->param.redGain = 1;
    return sk->status;
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
    if (strcmp(status, "RE") == 0){
        reply->replyResult = GRILL_OK;
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
    }
    else {
        reply->replyResult = GRILL_ERROR;
    }
    printf("tpvParseReply() %s --> [%d,%s]:%s\n", rstr, reply->msgLength, tpvCmdName(reply->cmdName), reply->replyStr);
}
