/*
    PGM Bayer to PPM converter and detailed analysis tool
 */
#include "core/buffers/converters/debayerTool.h"

int main(int argc, const char **argv)
{
    return corecvs::DebayerTool().proceed(argc, argv);
}
