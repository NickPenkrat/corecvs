/**
 * \file radialCorrection.h
 * \brief Header file for the radial lens distortion correction primitives
 *
 * \ingroup cppcorefiles
 *
 * \date Jun 22, 2010
 * \author alexander
 */

#ifndef RADIALCORRECTION_H_
#define RADIALCORRECTION_H_

#include "lensDistortionModelParameters.h"
#include "global.h"
#include "vector2d.h"
#include "g12Buffer.h"
#include "rgb24Buffer.h"
#include "function.h"

namespace corecvs {


class RadialCorrection : public DeformMap<int32_t, double>, public FunctionArgs
{
public:
    explicit RadialCorrection(const LensDistortionModelParameters &params = LensDistortionModelParameters());

    virtual ~RadialCorrection();

    inline Vector2dd map(Vector2dd const & v) const
    {
        return map(v.y(), v.x());
    }

    inline double radialScaleNormalized(double r) const
    {
        double rpow = r;
        double radialCorrection = 0;

        for (unsigned i = 0; i < mParams.koeff().size(); i ++)
        {
            radialCorrection += mParams.koeff()[i] * rpow;
            rpow *= r;
        }
        return radialCorrection;
    }

    inline double radialScale(double r) const
    {
        double normalizedR = r / mParams.normalizingFocal();
        return radialScaleNormalized(normalizedR);
    }


    inline Vector2dd map(int y, int x) const
    {
        return map((double)y, (double)x);
    }

    /**
     * \brief Apply the lens distortion correction.
     *
     * For details see LensDistortionModelParameters
     *
     * \param y  y position of the point to be mapped
     * \param x  x position of the point to be mapped
     * \return A corrected buffer
     *
     **/
    inline Vector2dd map(double y, double x) const
    {
        double cx = mParams.principalX();
        double cy = mParams.principalY();
        double p1 = mParams.tangentialX();
        double p2 = mParams.tangentialY();

        double dx = (x - cx) / mParams.normalizingFocal() * mParams.aspect();
        double dy = (y - cy) / mParams.normalizingFocal();

        /*double dx = dpx / mParams.focal;
        double dy = dpy / mParams.focal;*/

        double dxsq = dx * dx;
        double dysq = dy * dy;
        double dxdy = dx * dy;

        double rsq = dxsq + dysq;
        double r = sqrt(rsq);

        double radialCorrection = radialScaleNormalized(r);
//        SYNC_PRINT(("RadialCorrection::map (): [%lf %lf ] %lf %lf\n", x, y, rsq, radialCorrection));


        double radialX = (double)dx * radialCorrection;
        double radialY = (double)dy * radialCorrection;

        double tangentX =    2 * p1 * dxdy      + p2 * ( rsq + 2 * dxsq );
        double tangentY = p1 * (rsq + 2 * dysq) +     2 * p2 * dxdy      ;

        return Vector2dd(
                cx + ((dx + radialX + tangentX) / mParams.aspect() * mParams.scale() * mParams.normalizingFocal()),
                cy + ((dy + radialY + tangentY)                    * mParams.scale() * mParams.normalizingFocal())
               );
    }

    /**/
    virtual void operator()(const double in[], double out[])
    {
        Vector2dd result = map(in[1], in[0]);
        out[1] = result.y();
        out[0] = result.x();
    }


    //G12Buffer *correctLens(G12Buffer *input);
    G12Buffer   *doCorrectionTransform(G12Buffer *inputBuffer);
    RGB24Buffer *doCorrectionTransform(RGB24Buffer *inputBuffer);

    //Vector2dd getCorrectionForPoint(Vector2dd input);
    LensDistortionModelParameters mParams;

    Vector2dd center() const
    {
        return Vector2dd(mParams.principalX(), mParams.principalY());
    }


    RadialCorrection invertCorrection(int h, int w, int step);
};


} //namespace corecvs
#endif /* RADIALCORRECTION_H_ */

