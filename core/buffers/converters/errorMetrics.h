/**
* \file errorMetrics.h
* \brief Error metrics definitions.
*
* \ingroup cppcorefiles
* \date    Nov 25, 2015
* \author  pavel.vasilev
*/

#include "global.h"
#include "rgbTBuffer.h"
#include "g12Buffer.h"

namespace corecvs
{

class ErrorMetrics
{
public:

    /**
    * Peak signal to noise ratio in 2 images, one being the original and the other being estimated.
    * The orded in which images are passed does not affect the result.
    *
    * \param   img1 Image A
    * \param   img2 Image B
    *
    * \return  Peak signal to noise ratio in decibels.
    */
    static double psnr(RGB48Buffer *img1, RGB48Buffer *img2);
    static double psnr(G12Buffer *img1, G12Buffer *img2);

    /**
    * Root-mean-square deviation in 2 images, one being the original and the other being estimated.
    * The orded in which images are passed does not affect the result.
    *
    * \param   img1 Image A
    * \param   img2 Image B
    *
    * \return  Root-mean-square deviation.
    */
    static double rmsd(RGB48Buffer *img1, RGB48Buffer *img2);
    static double rmsd(G12Buffer *img1, G12Buffer *img2);

    /**
    * Mean square error in 2 images, one being the original and the other being estimated.
    * The orded in which images are passed does not affect the result.
    *
    * \param   img1 Image A
    * \param   img2 Image B
    *
    * \return  Mean square error.
    */
    static double mse(RGB48Buffer *img1, RGB48Buffer *img2);
    static double mse(G12Buffer *img1, G12Buffer *img2);

};
}