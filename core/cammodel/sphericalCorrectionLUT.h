#ifndef SPHERICALCORRECTIONLUT_H_
#define SPHERICALCORRECTIONLUT_H_
/**
 * \file sphericalCorrectionLUT.h
 * \brief Add Comment Here
 *
 * \date Feb 22, 2011
 * \author alexander
 */

#include <type_traits>
#include <vector>

#include "abstractBuffer.h"
#include "vector2d.h"
#include "deformMap.h"

namespace corecvs {

using std::vector;


template<class RadiusDeformer>
class SphericalCorrectionSq : public DeformMap<int32_t, double>
{
public:
    RadiusDeformer *radiusDeformer;
    Vector2dd center;

    SphericalCorrectionSq(const Vector2dd &_center, RadiusDeformer *_radiusDeformer) :
        radiusDeformer(_radiusDeformer),
        center(_center)
    {}

    Vector2d<double> map(const int32_t &y, const int32_t &x) const
    {
        return map(Vector2dd(x,y));
    }

    Vector2d<double> map(const Vector2d<double> &point) const
    {
        Vector2dd delta = point - center;
        double r2 = delta.sumAllElementsSq();
        double r2new = radiusDeformer->transformRadiusSquare(r2);
        return center + r2new * delta;
    }


};

template<class RadiusDeformer>
class SphericalCorrection : public DeformMap<int32_t, double>
{
public:
    RadiusDeformer *radiusDeformer;
    Vector2dd center;

    SphericalCorrection(const Vector2dd &_center, RadiusDeformer *_radiusDeformer) :
        radiusDeformer(_radiusDeformer),
        center(_center)
    {}

    Vector2d<double> map(const int32_t &y, const int32_t &x) const
    {
        return map(Vector2dd(x,y));
    }

    Vector2d<double> map(const Vector2d<double> &point) const
    {
        Vector2dd delta = point - center;
        double r2 = delta.sumAllElementsSq();
        double r2new = radiusDeformer->transformRadius(sqrt(r2));
        return center + r2new * delta;
    }

};

class RadiusCorrectionLUTSq
{
public:
    const vector<Vector2dd> *LUT;
    RadiusCorrectionLUTSq( const vector<Vector2dd> *_LUT) : LUT(_LUT) {}


    inline double transformRadiusSquare(double rSquare) const
    {
        unsigned n = 1;

        for (; n + 2 < LUT->size(); n++)
        {
            if (LUT->operator[](n + 1).x() > rSquare)
                break;
        }

        double x0 = LUT->operator[](n    ).x();
        double y0 = LUT->operator[](n    ).y();
        double x1 = LUT->operator[](n + 1).x();
        double y1 = LUT->operator[](n + 1).y();

         /**
          * Liner interpolation
          *
          *     1
          *  ------- * ((y2 - y1) * r^2 + y1 * x2 - y2 * x1) =
          *  x2 - x1
          *
          *
          *  y2 - y1
          *  ------- * (r^2 - x1) + y1
          *  x2 - x1
          **/
         //return (1.0 / (x1 - x0)) * ((y1 - y0) * rSquare + y0 * x1 - y1 * x0);
         return lerp(y0, y1, rSquare, x0, x1);
         //return ((y1 - y0) / (x1 - x0)) * (rSquare - x0) + y0;
    }
};


class RadiusCorrectionLUT
{
public:
    vector<double> LUT;
    RadiusCorrectionLUT( const vector<Vector2dd> &_LUT)
    {
       double last = sqrt(_LUT.back().x());
       LUT.reserve(last + 1);
       for (int i = 0; i < last; i++)
       {
           double val = i;
           double rSquare = val * val;
           size_t n = 1;
           for (; n + 2 < _LUT.size(); n++)
           {
               if (_LUT[n + 1].x() > rSquare)
                   break;
           }
           double koef = lerp(_LUT[n].y(), _LUT[n + 1].y(), rSquare, _LUT[n].x(), _LUT[n + 1].x());
           LUT.push_back(koef);
       }
    }


    inline double transformRadius(double radius) const
    {
        if (radius < LUT.size() - 1 && radius > 0)
        {
            int n = radius;
            double y0 = LUT[n    ];
            double y1 = LUT[n + 1];
            return lerp(y0, y1, radius, n, n+1);
        }
    }
};

class SphericalCorrectionLUTSq : public SphericalCorrectionSq<RadiusCorrectionLUTSq>
{
public:
    SphericalCorrectionLUTSq(const Vector2dd &_center, RadiusCorrectionLUTSq *_radiusDeformer) :
        SphericalCorrectionSq<RadiusCorrectionLUTSq>(_center, _radiusDeformer)
     {

     }
};

class SphericalCorrectionLUT : public SphericalCorrection<RadiusCorrectionLUT>
{
public:
    SphericalCorrectionLUT(const Vector2dd &_center, RadiusCorrectionLUT *_radiusDeformer) :
        SphericalCorrection<RadiusCorrectionLUT>(_center, _radiusDeformer)
     {

     }
};


} //namespace corecvs
#endif /* SPHERICALCORRECTIONLUT_H_ */

