#ifndef AVENCODER_H
#define AVENCODER_H

#include "core/buffers/rgb24/rgb24Buffer.h"

extern "C" {
#include <libavutil/frame.h>
#include <libavcodec/avcodec.h>
}

class AVEncoder
{
public:
    AVEncoder();
    bool open = false;

    int startEncoding(const std::string &name, int h, int w, int codec_id = -1);
    void addFrame(corecvs::RGB24Buffer *input);
    void endEncoding();

    static void printCaps();

protected:
    int got_output = 0;
    int frame_number = 0;

    AVFrame *frame = NULL;
    AVCodecContext *context = NULL;

    FILE *outFile = NULL;
};

#endif // AVENCODER_H
