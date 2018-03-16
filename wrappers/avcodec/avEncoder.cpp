#include "avEncoder.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
}

AVEncoder::AVEncoder()
{

}

int AVEncoder::startEncoding(const std::string &name, int h, int w, int codec_id )
{
    avcodec_register_all();

    if (codec_id == -1) {
        codec_id = AV_CODEC_ID_MPEG4;
    }

    AVCodec *codec = NULL;
    int ret;

    printf("AVEncoder::startEncoding(%s):called\n", name.c_str());

    /* find the mpeg1 video encoder */
    codec = avcodec_find_encoder((AVCodecID)codec_id);
     if (!codec) {
         fprintf(stderr, "Codec not found\n");
         return 1;
     }

     context = avcodec_alloc_context3(codec);
     if (!context) {
         fprintf(stderr, "Could not allocate video codec context\n");
         return 1;
     }

     /* put sample parameters */
     context->bit_rate = 400000;
     /* resolution must be a multiple of two */
     context->width = w;
     context->height = h;
     /* frames per second */
     context->time_base= (AVRational){1,25};
     context->gop_size = 10; /* emit one intra frame every ten frames */
     context->max_b_frames=1;
     context->pix_fmt = AV_PIX_FMT_YUV420P;

     if(codec_id == AV_CODEC_ID_H264)
         av_opt_set(context->priv_data, "preset", "slow", 0);

     /* open it */
     if (avcodec_open2(context, codec, NULL) < 0) {
         fprintf(stderr, "Could not open codec\n");
         return 1;
     }

     outFile = fopen(name.c_str(), "wb");
     if (outFile == NULL) {
         fprintf(stderr, "Could not open %s\n", name.c_str());
         return 1;
     }

     frame = av_frame_alloc();
     if (!frame) {
         fprintf(stderr, "Could not allocate video frame\n");
         return 1;
     }
     frame->format = context->pix_fmt;
     frame->width  = context->width;
     frame->height = context->height;

     /* the image can be allocated by any means and av_image_alloc() is
      * just the most convenient way if av_malloc() is to be used */
     ret = av_image_alloc(frame->data, frame->linesize, context->width, context->height,
                          context->pix_fmt, 32);
     if (ret < 0) {
         fprintf(stderr, "Could not allocate raw picture buffer\n");
         exit(1);
     }

     open = true;
     return 0;
}

void AVEncoder::addFrame(corecvs::RGB24Buffer *input)
{
    int ret;

    if (input == NULL)
        return;

    AVPacket pkt;
    av_init_packet(&pkt);
    pkt.data = NULL;    // packet data will be allocated by the encoder
    pkt.size = 0;

     fflush(stdout);
     /* prepare a dummy image */
     /* Y */
     for(int  y = 0; y < context->height; y++) {
         for(int x = 0; x < context->width; x++) {
             frame->data[0][y * frame->linesize[0] + x] = input->element(y,x).y();
         }
     }

     /* Cb and Cr */
     for(int y = 0; y < context->height / 2; y++) {
         for(int x = 0; x < context->width / 2; x++) {
             int x2 = x * 2;
             int y2 = y * 2;

             int cb = input->element(y2    , x2    ).cb() +
                      input->element(y2 + 1, x2    ).cb() +
                      input->element(y2    , x2 + 1).cb() +
                      input->element(y2 + 1, x2 + 1).cb();
             cb = cb / 4;

             int cr = input->element(y2    , x2    ).cr() +
                      input->element(y2 + 1, x2    ).cr() +
                      input->element(y2    , x2 + 1).cr() +
                      input->element(y2 + 1, x2 + 1).cr();
             cr = cr / 4;

             frame->data[1][y * frame->linesize[1] + x] = cr;
             frame->data[2][y * frame->linesize[2] + x] = cb;
         }
     }

     frame->pts = frame_number;

     /* encode the image */
     ret = avcodec_encode_video2(context, &pkt, frame, &got_output);
     if (ret < 0) {
         fprintf(stderr, "Error encoding frame\n");
         exit(1);
     }

     if (got_output) {
         printf("Write frame %3d (size=%5d)\n", frame_number, pkt.size);
         fwrite(pkt.data, 1, pkt.size, outFile);
         av_free_packet(&pkt);
     }

     /* get the delayed frames */
     for (got_output = 1; got_output; frame_number++) {
         fflush(stdout);

         ret = avcodec_encode_video2(context, &pkt, NULL, &got_output);
         if (ret < 0) {
             fprintf(stderr, "Error encoding frame\n");
             exit(1);
         }

         if (got_output) {
             printf("Write frame %3d (size=%5d)\n", frame_number, pkt.size);
             fwrite(pkt.data, 1, pkt.size, outFile);
             av_free_packet(&pkt);
         }
     }

}

void AVEncoder::endEncoding()
{
    int ret;
    uint8_t endcode[] = { 0, 0, 1, 0xb7 };



    /* add sequence end code to have a real mpeg file */
    fwrite(endcode, 1, sizeof(endcode), outFile);
    fclose(outFile);
    outFile = NULL;

    avcodec_close(context);
    av_free(context);
    av_freep(&frame->data[0]);
    av_frame_free(&frame);
    printf("\n");
    open = false;
}

void AVEncoder::printCaps()
{
    SYNC_PRINT(("AVEncoder::printCaps():codec list\n"));
    avcodec_register_all();

    AVCodec *codec = NULL;
    while (true)
    {
        codec = av_codec_next(codec);
        if (codec == NULL)
        {
            break;
        }
        SYNC_PRINT(("0x%X %s %s\n", codec->id, codec->name, codec->long_name));
    }
    SYNC_PRINT(("=============\n"));

}
