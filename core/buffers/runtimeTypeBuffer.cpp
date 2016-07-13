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

