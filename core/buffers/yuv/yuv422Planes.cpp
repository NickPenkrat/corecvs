#include "core/buffers/yuv/yuv422Planes.h"

namespace corecvs  {

YUV422Planes::YUV422Planes() :
    yPlane(10,10),
    uPlane(10,10),
    vPlane(10,10)
{
}

YUV422Planes YUV422Planes::FromRGB24(RGB24Buffer *input)
{

}

RGB24Buffer *YUV422Planes::toRGB24()
{

}

}

