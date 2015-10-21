/**
 * \file ppmLoader.h
 * \brief This is a header for PPM file format reader
 *
 * \ingroup cppcorefiles
 * \date    Oct 21, 2015
 * \author  alexander
 * \author  pavel.vasilev
 */
#ifndef PPMLOADER_H_
#define PPMLOADER_H_

#include <string>

#include "global.h"

#include "bufferLoader.h"
#include "g12Buffer.h"
#include "rgb24Buffer.h"
#include "metamap.h"

namespace corecvs
{
    class PPMLoader : public BufferLoader<G12Buffer>
    {
    public:
        PPMLoader() {}
        virtual ~PPMLoader() {}

        virtual bool acceptsFile(string name);

        virtual G12Buffer * load(string name);

        /**
         * Load method overload.
         *
         * \author  pavel.vasilev
         * \date    Oct 21, 2015
         *
         * \param   name            File to load.
         * \param [out] metadata    If non-null, this will be filled by any metadata encountered in PPM
         *                          file.
         *
         * \return  null if it fails, else a G12Buffer*.
         */

        G12Buffer * load(const string& name, MetaData *metadata);

        G12Buffer* g12BufferCreateFromPGM(const string& name, MetaData *metadata = nullptr);
        G12Buffer* g16BufferCreateFromPPM(const string& name);

        /**
         * Save method overloads.
         *
         * \author  pavel.vasilev
         * \date    Oct 21, 2015
         *
         * \param   name            Output file name.
         * \param [in]  buffer      Buffer to write to file. Internally writes graymap .pgm for G12 and
         *                          RGB .ppm for RGB.
         * \param [in]  metadata    (Optional) If non-null, writes metadata to file.
         *
         * \return  Error code.
         */

        int save(const string& name, G12Buffer *buffer, MetaData* metadata = nullptr);
        int save(const string& name, RGB24Buffer *buffer, MetaData* metadata = nullptr);
        int save(const string& name, RGB48Buffer *buffer, MetaData* metadata = nullptr);

        // TODO: remove this. 
        int saveG16(const string& name, G12Buffer *buffer);

    private:
        static string prefix1, prefix2;

        char* nextLine(FILE *fp, int sz, MetaData *metadata);
        bool readHeader(FILE *fp, unsigned long int *h, unsigned long int *w, unsigned short int *maxval, uint8_t *type, MetaData* metadata);
        bool writeHeader(FILE *fp, unsigned long int h, unsigned long int w, uint8_t type, uint64_t maxval, MetaData* metadata);

    };

} //namespace corecvs

#endif 
