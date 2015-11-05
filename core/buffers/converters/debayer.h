/**
 * \file    buffers\converters\debayer.h
 *
 * Declares the debayer class.
 */

#ifndef CDEBAYER_H_
#define CDEBAYER_H_

#include "global.h"
#include "g12Buffer.h"
#include "rgbTBuffer.h"
#include "rgb24Buffer.h"
#include "metamap.h"


namespace corecvs
{

class Debayer
{
public:

    enum Quality
    {
        Nearest = -5,
        Bilinear = 0,
        AHD = 3,
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

private:
    G12Buffer* mBayer = nullptr;
    int mDepth = 12;
    MetaData *mMetadata = nullptr;
    uint16_t* mCurve = nullptr;
    Vector3dd mScaleMul;
    uint16_t mBlack = 0;
    uint8_t mBayerPos = 3;

    void scaleCoeffs();
    uint16_t* gammaCurve(int imax);
    void preprocess(bool overwrite = false);

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
     * Clamp the given value. If a &lt; b, treat a as left limit and b as right. Invert limits
     * otherwise.
     *
     * \param   x   Value.
     * \param   a   Limit.
     * \param   b   Limit.
     *
     * \return  Clamped value.
     */

    static int32_t clamp(int32_t x, int32_t a, int32_t b);

    /**
     * Calculate weighted average.
     *
     * \param   coords  The coordinates to average pixels at. If the coordinate exceeds image
     *                  boundaries, it will be ignored.
     * \param   coeffs  Weights.
     *
     * \return  Averaged value.
     */

    int32_t weightedBayerAvg(vector<Vector2d32> coords, vector<int>coeffs = vector<int>());
    int32_t weightedBayerAvg(Vector2d32 coords);
    inline uint8_t colorFromBayerPos(int i, int j, bool rggb = true);
public:
    G12Buffer* median(G12Buffer* in);
};

}
#endif