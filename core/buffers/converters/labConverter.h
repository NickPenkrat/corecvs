/**
* \file labConverter.h
* \brief RGB to Lab converter definitions
*
* \ingroup cppcorefiles
* \date    Nov 5, 2015
* \author  pavel.vasilev
*/

#ifndef LABCONVERTER_H_
#define LABCONVERTER_H_

#include "global.h"
#include "rgbTColor.h"
#include "rgbColor.h"

namespace corecvs {

class LabConverter
{

private:
    static const float lab_eps;
    static const float lab_coeff;

    static const float srgb2xyz[3][3];

    static inline float GetLabValueL(float Y, float fy)
    {
        if (Y <= lab_eps)                                       // eps = 0.008856451586
            return lab_coeff * Y;                               // if linear functionality was used
        return 116 * fy - 16;                                   // if it was used after the cubic root
    }

    static inline float GetCubeRoot(float t)
    {
        if (t <= lab_eps)
        {                                                 // = 0.008856451586
            return (lab_coeff * t + 16) / 116;            // = ( 903.3 * t + 16 ) / 116
        }
        else
        {
            return cbrt(t);
        }
    }

    static inline void xyz2Lab(float* xyz, float* Lab)
    {
        float fx = GetCubeRoot(xyz[0]);
        float fy = GetCubeRoot(xyz[1]);
        float fz = GetCubeRoot(xyz[2]);

        Lab[0] = GetLabValueL(xyz[1], fy);                  // L
        Lab[1] = float(500) * (fx - fy);                    // a
        Lab[2] = float(200) * (fy - fz);                    // b
    }

public:

    /**
    * Convert RGB to CIELab.
    *
    * \param   RGB RGB pixel
    * \param   Lab CIELab pixel
    *
    */
    template <typename T>
    static inline void rgb2Lab(RGBTColor<T> RGB, float* Lab)
    {
        const float max = (1 << sizeof(T)) - 1;

        float xyz[3];

        for (int i = 0; i < 3; i++)
            xyz[i] = (srgb2xyz[i][0] * RGB.r() + srgb2xyz[i][1] * RGB.g() + srgb2xyz[i][2] * RGB.b()) / max;

        xyz2Lab(xyz, Lab);

    }

    static inline void rgb2Lab(RGBColor RGB, float* Lab)
    {
        const float max = 255;

        float xyz[3];

        for (int i = 0; i < 3; i++)
            xyz[i] = (srgb2xyz[i][0] * RGB.r() + srgb2xyz[i][1] * RGB.g() + srgb2xyz[i][2] * RGB.b()) / max;

        xyz2Lab(xyz, Lab);

    }
};


} // namespace corecvs

const float LabConverter::lab_eps(0.008856451586f);
const float LabConverter::lab_coeff(903.3f);
const float LabConverter::srgb2xyz[3][3] {
    { 0.412453f, 0.357585f, 0.180423f },
    { 0.212671f, 0.715160f, 0.072169f },
    { 0.019334f, 0.119193f, 0.950227f }
};

#endif //LABCONVERTER_H_