#ifndef LIBJPEGFILEREADER_H
#define LIBJPEGFILEREADER_H

#include "rgb24Buffer.h"
#include "bufferFactory.h"


class LibjpegFileReader : public corecvs::BufferLoader<corecvs::RGB24Buffer>
{
    static std::string prefix1;
    static std::string prefix2;
public:
    static int registerMyself()
    {
        corecvs::BufferFactory::getInstance()->registerLoader(new LibjpegFileReader());
        return 0;
    }

    virtual bool acceptsFile(std::string name) override;
    virtual corecvs::RGB24Buffer *load(std::string name) override;

    LibjpegFileReader();
    virtual ~LibjpegFileReader();
};

#endif // LIBJPEGFILEREADER_H
