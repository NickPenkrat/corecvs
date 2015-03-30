#include <stdio.h>
#include <QtCore/QCoreApplication>
#include <QtCore/QThread>
#include <unistd.h>

#include "decoupleYUYV.h"
#include "bmpLoader.h"

int main (int argc, char **argv)
{    
    const char *fileName = "data/x.raw";
    if (argc >= 2)
        fileName = argv[1];

    FILE *In = fopen(fileName, "rb");
    if (In == NULL)
    {
         printf("Can't open the file: <%s>\n", fileName);
         return 1;
    }

    const int formatH = 1080;
    const int formatW = 1920 * 2 * 2;
    const int dataSize = formatH * formatW;

    uint8_t *ptr = new uint8_t[dataSize];
    if (fread(ptr, sizeof(uint8_t), dataSize, In) != dataSize)
    {
        printf("File to small: <%s>\n", fileName);
        return 1;
    }

    ImageCaptureInterface::FramePair result;

    DecoupleYUYV::decouple(formatH, formatW, ptr, DecoupleYUYV::SIDEBYSIDE_SYNCCAM_1, result);

    if (result.bufferLeft != NULL) {
        BMPLoader().save("leftg12.bmp", result.bufferLeft);
    }
    if (result.bufferRight != NULL) {
        BMPLoader().save("rightg12.bmp", result.bufferRight);
    }

    if (result.rgbBufferLeft != NULL) {
        BMPLoader().save("left24.bmp", result.rgbBufferLeft);
    }
    if (result.rgbBufferRight != NULL) {
        BMPLoader().save("right24.bmp", result.rgbBufferRight);
    }

    deletearr_safe (ptr);
    delete_safe(result.bufferLeft);
    delete_safe(result.bufferRight);
    delete_safe(result.rgbBufferLeft);
    delete_safe(result.rgbBufferRight);
    fclose(In);

	return 0;
}
