#ifndef LIBPNGFILEREADER_H
#define LIBPNGFILEREADER_H

#include <string>
#include <stdint.h>

#include "global.h"


#include "bufferLoader.h"
#include "bufferFactory.h"
#include "g12Buffer.h"
#include "rgb24Buffer.h"

using std::string;

class LibpngFileReader : public corecvs::BufferLoader<corecvs::RGB24Buffer>
{
    static string prefix1;

public:
    static int registerMyself()
    {
        corecvs::BufferFactory::getInstance()->registerLoader(new LibpngFileReader());
        return 0;
    }

    virtual bool acceptsFile(corecvs::string name) override;
    virtual corecvs::RGB24Buffer * load(string name) override;
    virtual bool save(corecvs::string name, corecvs::RGB24Buffer *buffer);

};


#endif // LIBPNGFILEREADER_H

