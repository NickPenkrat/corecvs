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
public:
    static string prefix1;

    static int registerMyself()
    {
        corecvs::BufferFactory::getInstance()->registerLoader(new LibpngFileReader());
        return 0;
    }

    virtual bool acceptsFile(corecvs::string name) override;
    virtual corecvs::RGB24Buffer * load(string name) override;
    virtual std::string name() override { return "LibPNG"; }
    virtual bool save(corecvs::string name, corecvs::RGB24Buffer *buffer);

};

class LibpngFileSaver : public corecvs::BufferSaver<corecvs::RGB24Buffer>
{
    virtual bool acceptsFile(string name) {
        return LibpngFileSaver::acceptsFile(name);
    }
    virtual bool save(corecvs::RGB24Buffer &buffer, string name) override {
        return LibpngFileReader().save(name, &buffer);
    }

    virtual std::string              name()        override { return "LibpngFileSaver"; }
    virtual std::vector<std::string> resolutions() override {
        return std::vector<std::string>({LibpngFileReader::prefix1});
    }
    virtual ~LibpngFileSaver() {}
};



#endif // LIBPNGFILEREADER_H

