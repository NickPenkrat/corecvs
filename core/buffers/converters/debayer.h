/**
 * \file    buffers\converters\debayer.h
 *
 * Declares the debayer class.
 */

#ifndef CDEBAYER_H_
#define CDEBAYER_H_

#include "g12Buffer.h"
#include "rgb24Buffer.h"
#include "metamap.h"

using namespace std;
using corecvs::G12Buffer;
class Debayer
{
private:
    G12Buffer* mBayer = nullptr;
    int mDepth = 12;
    MetaData *mMetadata = nullptr;
    uint16_t* mCurve = nullptr;
    double* mScaleMul = nullptr;
    uint16_t mBlack = 0;
    double* scaleCoeffs();

    uint16_t* gammaCurve(int mode, int imax);
    void preprocess(bool overwrite = false);

    RGB48Buffer* linear();
    RGB48Buffer* nearest();

    uint16_t clip(int64_t x, int depth = 16);

public:

    enum Quality
    {
        Nearest = -5,
        Bilinear = 0,

    };

    /**
     * Constructor.
     *
     * \author pavel.vasilev
     * \date 21 Oct 2015
     *
     * \param [in,out] bayer Raw bayer data.
     * \param depthOut       Processing bit depth.
     * \param [in,out] data  (Optional) Metadata.
     */

    Debayer(G12Buffer *bayer, int depthOut = 12, MetaData *data = nullptr);

    /**
     * Converts the image to RGB48.
     *
     * \author  pavel.vasilev
     * \date    21 oct 2015
     *
     * \param   quality Demosaic quality (method).
     *
     * \return  Resulting image.
     */

    RGB48Buffer* toRGB48(Quality quality);

    ~Debayer();
};

#endif