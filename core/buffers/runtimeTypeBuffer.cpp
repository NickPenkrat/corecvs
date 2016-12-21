#include "runtimeTypeBuffer.h"

#include <iomanip>

void corecvs::RuntimeTypeBuffer::load(std::istream &is)
{
    is >> (*this);
}

void corecvs::RuntimeTypeBuffer::save(std::ostream &os) const
{
    os << (*this);
}

std::istream& operator>>(std::istream &is, corecvs::RuntimeTypeBuffer &b)
{
    size_t R, C;
    std::string type;
    int buffer_type = corecvs::BufferType::U8;
    is >> R >> C >> type;
    if(type == "F32")
    {
        buffer_type = corecvs::BufferType::F32;
    }

    b = corecvs::RuntimeTypeBuffer(R, C, buffer_type);
    switch(b.type)
    {
        case corecvs::BufferType::U8:
            for(size_t i = 0; i < R; ++i)
                for(size_t j = 0; j < C; ++j)
                {
                    int v;
                    is >> v;
                    b.at<uint8_t>(i, j) = v;
                }
            break;
        case corecvs::BufferType::F32:
            for(size_t i = 0; i < R; ++i)
                for(size_t j = 0; j < C; ++j)
                {
                    is >> b.at<float>(i, j);
                }
            break;
    }
    return is;
}

std::ostream& operator<<(std::ostream &os, const corecvs::RuntimeTypeBuffer &b)
{
    size_t R = b.rows, C = b.cols;
    std::string type = "U8";
    if(b.type == corecvs::BufferType::F32)
    {
        type = "F32";
    }

    os << R << " " << C << " " << type << std::endl;
    switch(b.type)
    {
        case corecvs::BufferType::U8:
            for(size_t i = 0; i < R; ++i)
            {
                for(size_t j = 0; j < C; ++j)
                {
                    int v = b.at<uint8_t>(i, j);
                    os << std::setw(5) << v;
                }
                os << std::endl;
            }
            break;
        case corecvs::BufferType::F32:
            os << std::setprecision(15) << std::scientific;
            for(size_t i = 0; i < R; ++i)
            {
                for(size_t j = 0; j < C; ++j)
                {
                    os << std::setw(20) << b.at<float>(i, j) << "\t";
                }
                os << std::endl;
            }
            break;
    }
    return os;
}

#include "g12Buffer.h"
#include "g8Buffer.h"
#include "rgb24Buffer.h"

namespace corecvs {

/* Converters */
G8Buffer *RuntimeTypeBuffer::toG8Buffer()
{
    if (!isValid())
        return NULL;
    G8Buffer *buffer = new G8Buffer(rows, cols);
    if (type == BufferType::U8)
    {
        buffer->fillWithRaw(data);
    }

    if (type == BufferType::F32)
    {

        for (size_t i = 0; i < rows; i++)
        {
            for (size_t j = 0; j < cols; j++)
            {
                buffer->element(i,j) = at<float>(i,j);
            }
        }
    }
    return buffer;
}

G12Buffer *RuntimeTypeBuffer::toG12Buffer(double min, double max)
{
    if (!isValid())
        return NULL;
    G12Buffer *buffer = new G12Buffer(rows, cols);
    if (type == BufferType::U8)
    {
        for (size_t i = 0; i < rows; i++)
        {
            for (size_t j = 0; j < cols; j++)
            {
                buffer->element(i,j) = at<uint8_t>(i,j) << 4;
            }
        }
    }

    if (type == BufferType::F32)
    {
        for (size_t i = 0; i < rows; i++)
        {
            for (size_t j = 0; j < cols; j++)
            {
                double value = lerpLimit(0, G12Buffer::BUFFER_MAX_VALUE, at<float>(i,j), min, max);
                buffer->element(i,j) = (uint16_t)value;
            }
        }
    }
    return buffer;
}

RGB24Buffer *RuntimeTypeBuffer::toRGB24Buffer(double min, double max)
{
    if (!isValid())
        return NULL;
    RGB24Buffer *buffer = new RGB24Buffer(rows, cols);
    if (type == BufferType::U8)
    {
        for (size_t i = 0; i < rows; i++)
        {
            for (size_t j = 0; j < cols; j++)
            {
                buffer->element(i,j) = RGBColor::gray(at<uint8_t>(i,j));
            }
        }
    }

    if (type == BufferType::F32)
    {
        for (size_t i = 0; i < rows; i++)
        {
            for (size_t j = 0; j < cols; j++)
            {
                double value = lerpLimit(0, 0xFF, at<float>(i,j), min, max);
                buffer->element(i,j) = RGBColor::gray(value);
            }
        }
    }
    return buffer;
}

} //namespace corecvs
