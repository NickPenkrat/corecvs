#pragma once
/*
Converts .cr2 to .ppm or bayer non-filtered .ppm using LibRaw.
LibRaw should be modified to have its methods public
*/
#include <string>
#include "global.h"
#include "g12Buffer.h"
#include "metamap.h"

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
     * Write Bayer data with metadata to file.
     *
     * \author  pavel.vasilev
     * \date    Oct 21, 2015
     *
     * \param   filename    File to write to.
     *
     * \return  Error code.
     */

    int writeBayer(const string& filename);

    /**
    * Replaces LibRaw-unpacked Bayer data with arbitrary.
    *
    * \author  pavel.vasilev
    * \date    Oct 21, 2015
    *
    * \param [in,out]  img If non-null, the image.
    */

    void fakeimg(corecvs::G12Buffer *img);

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
    int flipIndex(int row, int col);
    void histUpdate(int i, int j, uint16_t val);
    uint64_t **hist = 0;
    LibRaw *reader = 0;
    int shift = 0;
    
};

