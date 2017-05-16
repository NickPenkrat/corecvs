#ifndef TPV_H
#define TPV_H

#ifdef __cplusplus
extern "C" {
#endif

enum TopViGrillResult {
    GRILL_ERROR,
    GRILL_OK
};

enum TopViCmdType {
    TPV_UNDEFINED = 0,
    TPV_GET,
    TPV_SET,
};

enum TopViCmdName {
    TPV_UNINITED = -1,
    TPV_INIT = 1,
    TPV_STATUS,
    TPV_GRAB,
    TPV_LIVE,
    TPV_FOCUS,
    TPV_CONTROL,
    TPV_EXPOSURE,
    TPV_GAIN,
    TPV_CLEAN
};

enum TopViCmdStatus{
    TPV_UNKNOWN = -1,
    TPV_QUERY,
    TPV_NO_REPLY,
    TPV_WAIT_REPLY,
    TPV_REPLY,
    TPV_OK
};

enum TopViGain {
    TPV_GLOBAL = 0,
    TPV_GREEN1,
    TPV_RED,
    TPV_BLUE,
    TPV_GREEN2
};

enum TopVi_STATE{
    TPV_SK_UNKNOWN = -1,
    TPV_SK_OK,
    TPV_SK_ERROR
};

enum TopVi_ACTIVE {
    TPV_PK_UNKNOWN = -1,
    TPV_PK_SLEEP,
    TPV_PK_LIVE,
    TPV_PK_GRAB
};

enum TopVi_SIZE {
    TPV_PK_3840x2780,
    TPV_PK_3664x2748,
    TPV_PK_1920x1080,
    TPV_PK_1408x792,
    TPV_PK_1280x720,
    TPV_PK_960x688,
    TPV_PK_896x688,
    TPV_PK_480x340
};

enum TopVi_BINNING {
    TPV_PK_1X1,
    TPV_PK_2X2,
    TPV_PK_4X4
};

struct TopVi_PK {
   enum TopVi_ACTIVE activeMode;
   enum TopVi_BINNING binningMode;
   enum TopVi_SIZE size;
   int width;
   int height;
   double exposure;
   double globalGain;
   double blueGain;
   double redGain;
};

struct TopVi_SK {
    enum TopVi_STATE status;
    int camId;
    struct TopVi_PK param;
};

struct TopVi_SU {
    enum TopVi_STATE status;
    int camNumber;
    struct TPV_SK *cameras;
};

struct grill_request {
    const char *requestStr;
};


struct grill_reply {
    enum TopViGrillResult replyResult;
    int msgLength;
    enum TopViCmdName cmdName;
    char *replyStr;
};

static inline const char *tpvCmdType(enum TopViCmdType cmdType){
    switch (cmdType) {
        case TPV_UNDEFINED: return "undefined";
        case TPV_GET: return "print";
        case TPV_SET: return "set";
    }
    return "UNKNOWN";
}

static inline const char *tpvCmdName(enum TopViCmdName cmdName){
    switch (cmdName) {
        case TPV_UNINITED: return "uninited";
        case TPV_INIT: return "restart";
        case TPV_STATUS: return "status";
        case TPV_GRAB: return "grab";
        case TPV_EXPOSURE: return "exposure";
        case TPV_GAIN: return "gain";
        case TPV_LIVE: return "live";
        case TPV_FOCUS: return "focus";
        case TPV_CONTROL: return "control";
        case TPV_CLEAN: return "clean";
    }
    return "unknown";
}

static inline const char *tpvGetColorGain(enum TopViGain gain) {
    switch (gain) {
        case TPV_GLOBAL: return "global";
        case TPV_BLUE: return "blue";
        case TPV_RED: return "red";
        default: return "global";
    }
    return "GLOBAL";
}

void tpvParseGrillReply(struct grill_reply *reply, const char *rstr);
const char *tpvGenerateCommandString(enum TopViCmdType cmdType, enum TopViCmdName cmdName, int camId, const char* value, const char* additional_value);
void tpvFreeString(const char *string);

enum TopViCmdStatus tpvCmdNeedReplay(enum TopViCmdType cmdType);
const char *tpvCmdStatus(enum TopViCmdStatus cmdStatus);
const char* tpvGenerateTestReply(enum TopViCmdType cmdType, enum TopViCmdName cmdName);
enum TopViGain tpvSetColorGain(const char *value);

#ifdef __cplusplus
}
#endif

#endif // TPV_H
