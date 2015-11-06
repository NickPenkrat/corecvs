/**
 * \file    buffers/converters/debayer.h
 *
 * Declares the Debayer class.
 */
#ifndef CDEBAYER_H_
#define CDEBAYER_H_

#include "global.h"

#include "g12Buffer.h"
#include "rgbTBuffer.h"
#include "rgb24Buffer.h"
#include "metamap.h"


namespace corecvs {

class Debayer
{
public:
    enum Quality    // TODO: is it quality or method ?
    {
        Nearest  = -5,
        Bilinear = 0,
        AHD      = 3,
        Improved = 7
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

    ~Debayer();

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

private:
    G12Buffer*  mBayer = nullptr;
    int         mDepth = 12;
    MetaData *  mMetadata = nullptr;
    uint16_t*   mCurve = nullptr;
    Vector3dd   mScaleMul;
    uint16_t    mBlack = 0;

    void        scaleCoeffs();
    uint16_t*   gammaCurve(int imax);
    void        preprocess(bool overwrite = false);

    RGB48Buffer* linear();
    RGB48Buffer* nearest();
    RGB48Buffer* improved();

    /* utilitary functions */
    /**
     * Clip int to uint16.
     *
     * \param   x       Value.
     * \param   depth   Bits to clip at.
     *
     * \return  Clipped value.
     */
    uint16_t clip(int32_t x, int depth = 16);

    /**
     * Calculate average.
     *
     * \param   coords  The coordinates to average pixels at. If the coordinate exceeds image
     *                  boundaries, it will be ignored.
     *
     * \return  Averaged value.
     */
    int32_t clampedBayerSum(Vector2d32 coords);
    int32_t clampedBayerSum(vector<Vector2d32> coords);
};

} // namespace corecvs

#endif // CDEBAYER_H_
