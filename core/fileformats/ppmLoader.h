#pragma once
/**
 * \file ppmLoader.h
 * \brief This is a header for PPM file format reader
 *
 * \ingroup cppcorefiles
 * \date Jun 22, 2010
 * \author alexander
 */

#include <string>
#include <map>

#include "global.h"

#include "bufferLoader.h"
#include "g12Buffer.h"

namespace corecvs {

    using std::string;

    class PPMLoader : public BufferLoader<G12Buffer>
    {

    public:
        typedef std::map<string, double*> MetaData;

        PPMLoader() {}
        virtual ~PPMLoader() {}

        virtual bool acceptsFile(string name);
        virtual G12Buffer * load(string name);
        G12Buffer * load(string name, MetaData *metadata);

        G12Buffer* g12BufferCreateFromPGM(string& name, MetaData *metadata = nullptr);
        G12Buffer* g16BufferCreateFromPPM(string& name);

        int save(string name, G12Buffer *buffer);
        int saveG16(string name, G12Buffer *buffer);

    private:
        static string prefix1, prefix2;

        char* nextLine(FILE *fp, int sz, MetaData *metadata);
        bool readHeader(FILE *fp, unsigned long int *h, unsigned long int *w, unsigned short int *maxval, int *type, MetaData* metadata);

    };

} //namespace corecvs
