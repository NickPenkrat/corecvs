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

#include "ellipticalApproximation.h"
#include "lensDistortionModelParameters.h"
#include "global.h"
#include "vector2d.h"
#include "g12Buffer.h"
#include "rgb24Buffer.h"
#include "function.h"
#include "levenmarq.h"

namespace corecvs {


class RadialCorrection : public DeformMap<int32_t, double>, public FunctionArgs
{
public:
    explicit RadialCorrection(const LensDistortionModelParameters &params = LensDistortionModelParameters());

    virtual ~RadialCorrection();

    inline Vector2dd map(const Vector2dd &v, bool fromUndistorted = false) const
    {
        return fromUndistorted ? mapFromUndistorted(v) : mapToUndistorted(v);
    }
    inline Vector2dd map(int y, int x, bool fromUndistorted = false) const
    {
        return fromUndistorted ? mapFromUndistorted(y, x) : mapToUndistorted(y, x);
    }
    inline Vector2dd map(double y, double x, bool fromUndistorted = false) const
    {
        return fromUndistorted ? mapFromUndistorted(y, x) : mapToUndistorted(y, x);
    }
    inline Vector2dd mapToUndistorted(Vector2dd const & v) const
    {
        return mapToUndistorted(v.y(), v.x());
    }
    inline Vector2dd mapFromUndistorted(Vector2dd const & v) const
    {
        return mapFromUndistorted(v.y(), v.x());
    }

    inline double radialScaleNormalized(double r) const
    {
        double rpow = r;
        double radialCorrection = 0;
        const vector<double>& coeffs = mParams.koeff();

        for (unsigned i = 0; i < coeffs.size(); i++)
        {
            radialCorrection += coeffs[i] * rpow;
            rpow *= r;
        }
        return radialCorrection;
    }

    inline double radialScale(double r) const
    {
        double normalizedR = r / mParams.normalizingFocal();
        return radialScaleNormalized(normalizedR);
    }


    inline Vector2dd mapFromUndistorted(int y, int x) const
    {
        return mapFromUndistorted((double)y, (double)x);
    }
    inline Vector2dd mapToUndistorted(int y, int x) const
    {
        return mapToUndistorted((double)y, (double)x);
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
    inline Vector2dd mapToUndistorted(double y, double x) const
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
               ) + Vector2dd(mParams.shiftX(), mParams.shiftY());
    }
    struct InverseFunctor : FunctionArgs
    {
        void operator() (const double* in, double *out)
        {
            Vector2dd x(in[0], in[1]);
            auto err = correction->map(x[1], x[0]) - target;
            out[0] = err[0];
            out[1] = err[1];
        }
        InverseFunctor(Vector2dd target, const RadialCorrection* correction) : FunctionArgs(2, 2), target(target), correction(correction)
        {
        }
        Vector2dd target;
        const RadialCorrection* correction;
    };
    Vector2dd mapFromUndistorted(double y, double x) const
    {
        InverseFunctor functor(Vector2dd(x, y), this);
        LevenbergMarquardt lm(1000);
        lm.f = &functor;
        std::vector<double> in(2), out(2);
        auto res = lm.fit(in, out);
        auto foo = !(map(res[1], res[0]) - Vector2dd(x, y));
        std::cout << foo << std::endl;
        CORE_ASSERT_TRUE_S(foo < 1.0);
        return Vector2dd(res[0], res[1]);
    }

    /**/
    virtual void operator()(const double in[], double out[])
    {
        Vector2dd result = mapToUndistorted(in[1], in[0]);
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
    EllipticalApproximation1d compareWith(const RadialCorrection &other, int h, int w, int steps);



    /* */
    void getCircumscribedImageRect(const int32_t &x1, const int32_t &y1, const int32_t &x2, const int32_t &y2,
                                   Vector2dd &min, Vector2dd &max)
    {
        min = mapToUndistorted(y1,x1);
        max = mapToUndistorted(y2,x2);

        for (int i = y1; i <= y2; i++)
        {
            Vector2dd mapLeft  = mapToUndistorted(i, x1);
            Vector2dd mapRight = mapToUndistorted(i, x2);
            if (mapLeft .x() < min.x()) min.x() = mapLeft.x();
            if (mapRight.x() > max.x()) max.x() = mapRight.x();
        }

        for (int j = x1; j <= x2; j++)
        {

            Vector2dd mapTop    = mapToUndistorted(y1, j);
            Vector2dd mapBottom = mapToUndistorted(y2, j);
            if (mapTop   .y() < min.y()) min.y() = mapTop.y();
            if (mapBottom.y() > max.y()) max.y() = mapBottom.y();
        }
    }

    void getInscribedImageRect(const int32_t &x1, const int32_t &y1, const int32_t &x2, const int32_t &y2,
                               Vector2dd &min, Vector2dd &max)
    {
        min = mapToUndistorted(y1,x1);
        max = mapToUndistorted(y2,x2);

        for (int i = y1; i <= y2; i++)
        {
            Vector2dd mapLeft  = mapToUndistorted(i, x1);
            Vector2dd mapRight = mapToUndistorted(i, x2);
            if (mapLeft .x() > min.x()) min.x() = mapLeft.x();
            if (mapRight.x() < max.x()) max.x() = mapRight.x();
        }

        for (int j = x1; j <= x2; j++)
        {

            Vector2dd mapTop    = mapToUndistorted(y1, j);
            Vector2dd mapBottom = mapToUndistorted(y2, j);
            if (mapTop   .y() > min.y()) min.y() = mapTop.y();
            if (mapBottom.y() < max.y()) max.y() = mapBottom.y();
        }
    }
};


} //namespace corecvs
#endif /* RADIALCORRECTION_H_ */

