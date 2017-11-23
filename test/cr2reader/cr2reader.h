#pragma once
/*
    Converts .cr2 to .ppm or bayer non-filtered .ppm using LibRaw.
    LibRaw should be modified to have its methods public
*/
#include <string>

#include "core/utils/global.h"

#include "core/buffers/g12Buffer.h"
#include "core/fileformats/metamap.h"

using std::string;

class LibRaw;

class CR2Reader
{
public:
    CR2Reader();
   ~CR2Reader();

    /**
     * Open file for reading.
     *
     * \author  pavel.vasilev
     * \date    Oct 21, 2015
     *
     * \param   filename    File to open.
     *
     * \return  Error code.
     */
    int open(const string& filename);

    /**
     * Get raw Bayer data from LibRaw-unpacked file.
     *
     * \author  pavel.vasilev
     * \date    Oct 21, 2015
     *
     * \param   shifted true if shifted.
     *
     * \return  null if it fails, else pointer to Bayer data.
     */
    corecvs::G12Buffer* getBayer(bool shifted = true);

    /**
     * Gets the metadata.
     *
     * \author  pavel.vasilev
     * \date    Oct 21, 2015
     *
     * \return  null if it fails, else the metadata.
     */
    corecvs::MetaData* getMetadata();

    /**
     * Replaces LibRaw-unpacked Bayer data with arbitrary.
     *
     * \author  pavel.vasilev
     * \date    Oct 21, 2015
     *
     * \param [in,out]  bayer   If non-null, the new Bayer data.
     */
    void fakeBayer(corecvs::G12Buffer *bayer);

    /**
     * Sets bits per pixel.
     *
     * \author  pavel.vasilev
     * \date    Oct 21, 2015
     *
     * \param   bits    The bit depth.
     */
    void setBPP(uint bits);

    /**
     * Sets the quality.
     *
     * \author  pavel.vasilev
     * \date    Oct 21, 2015
     *
     * \param   quality The quality.
     */
    void setQuality(uint quality);

    /**
    * Decode the image using LibRaw.
    *
    * \author  pavel.vasilev
    * \date    Oct 21, 2015
    *
    * \param   noWhiteBalance  true to disable wb adjustment.
    *
    * \return  Error code.
    */
    int processDCRaw(bool noWhiteBalance = true);

    /**
     * Output image data to PPM. Cannot use PPMLoader here because the data is in non-standard
     * format.
     *
     * \author  pavel.vasilev
     * \date    Oct 21, 2015
     *
     * \param   filename    File to write to.
     * \param   fullColour  Use full 16-bit color or reduced 8-bit. Ignored if input data is already
     *                      8-bit or less.
     *
     * \return  Error code.
     */
    int writePPM(const string& filename, bool fullColour = true);

private:
    int     flipIndex(int row, int col, bool raw = false);
    void    histUpdate(int i, int j, uint16_t val);

    uint64_t  **hist   = NULL;
    LibRaw     *reader = NULL;
    int         shift  = 0;
};
