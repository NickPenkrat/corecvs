/**
 * \file lensDistortionModelParameters.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "lensDistortionModelParameters.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace corecvs {
template<>
Reflection BaseReflection<LensDistortionModelParameters>::reflection = Reflection();
template<>
int BaseReflection<LensDistortionModelParameters>::dummy = LensDistortionModelParameters::staticInit();
} // namespace corecvs 

SUPPRESS_OFFSET_WARNING_BEGIN

int LensDistortionModelParameters::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "Lens Distortion Model Parameters",
        "Lens Distortion Model Parameters",
        ""
    );
     

    fields().push_back(
        new DoubleField
        (
          LensDistortionModelParameters::PRINCIPALX_ID,
          offsetof(LensDistortionModelParameters, mPrincipalX),
          240,
          "principalX",
          "principalX",
          "The center of the distortion \f$x_c\f$"
        )
    );
    fields().push_back(
        new DoubleField
        (
          LensDistortionModelParameters::PRINCIPALY_ID,
          offsetof(LensDistortionModelParameters, mPrincipalY),
          320,
          "principalY",
          "principalY",
          "The center of the distortion \f$y_c\f$"
        )
    );
    fields().push_back(
        new DoubleField
        (
          LensDistortionModelParameters::TANGENTIALX_ID,
          offsetof(LensDistortionModelParameters, mTangentialX),
          0,
          "tangentialX",
          "tangentialX",
          "First tangent correction coefficient - \f$p_1\f$"
        )
    );
    fields().push_back(
        new DoubleField
        (
          LensDistortionModelParameters::TANGENTIALY_ID,
          offsetof(LensDistortionModelParameters, mTangentialY),
          0,
          "tangentialY",
          "tangentialY",
          "Second tangent correction coefficient - \f$p_2\f$"
        )
    );
    fields().push_back(
        new DoubleVectorField
        (
          LensDistortionModelParameters::KOEFF_ID,
          offsetof(LensDistortionModelParameters, mKoeff),
          0,
          0,
          "koeff",
          "koeff",
          "Polynom to describe radial correction"
        )
    );
    fields().push_back(
        new DoubleField
        (
          LensDistortionModelParameters::ASPECT_ID,
          offsetof(LensDistortionModelParameters, mAspect),
          1,
          "aspect",
          "aspect",
          "aspect"
        )
    );
    fields().push_back(
        new DoubleField
        (
          LensDistortionModelParameters::SCALE_ID,
          offsetof(LensDistortionModelParameters, mScale),
          1,
          "scale",
          "scale",
          "scale"
        )
    );
    fields().push_back(
        new DoubleField
        (
          LensDistortionModelParameters::NORMALIZING_FOCAL_ID,
          offsetof(LensDistortionModelParameters, mNormalizingFocal),
          1,
          "Normalizing Focal",
          "Normalizing Focal",
          "Normalizing Focal"
        )
    );
    fields().push_back(
        new DoubleField
        (
          LensDistortionModelParameters::SHIFTX_ID,
          offsetof(LensDistortionModelParameters, mShiftX),
          0,
          "Shift X",
          "Shift X",
          "Shift X"
        )
    );
    fields().push_back(
        new DoubleField
        (
          LensDistortionModelParameters::SHIFTY_ID,
          offsetof(LensDistortionModelParameters, mShiftY),
          0,
          "Shift Y",
          "Shift Y",
          "Shift Y"
        )
    );
    fields().push_back(
        new BoolField
        (
          LensDistortionModelParameters::MAP_FORWARD_ID,
          offsetof(LensDistortionModelParameters, mMapForward),
          false,
          "Map forward",
          "Map forward",
          "True if maps from undistorted to distorted"
        )
    );
   return 0;
}

SUPPRESS_OFFSET_WARNING_END

void LensDistortionModelParameters::getInscribedImageRect(const Vector2dd &tlDistorted, const Vector2dd &drDistorted, Vector2dd &tlUndistorted, Vector2dd &drUndistorted) const
{
    std::vector<corecvs::Vector2dd> boundaries[4];
    getRectMap(tlDistorted, drDistorted, boundaries);

    tlUndistorted = mapBackward(tlDistorted);
    drUndistorted = mapBackward(drDistorted);

    for (auto& v: boundaries[0])
        if (v[1] > tlUndistorted[1])
            tlUndistorted[1] = v[1];
    for (auto& v: boundaries[2])
        if (v[1] < drUndistorted[1])
            drUndistorted[1] = v[1];
    for (auto& v: boundaries[1])
        if (v[0] > tlUndistorted[0])
            tlUndistorted[0] = v[0];
    for (auto& v: boundaries[3])
        if (v[0] < drUndistorted[0])
            drUndistorted[0] = v[0];
}

void LensDistortionModelParameters::getCircumscribedImageRect(const Vector2dd &tlDistorted, const Vector2dd &drDistorted, Vector2dd &tlUndistorted, Vector2dd &drUndistorted) const
{
    std::vector<corecvs::Vector2dd> boundaries[4];
    getRectMap(tlDistorted, drDistorted, boundaries);

    tlUndistorted = mapBackward(tlDistorted);
    drUndistorted = mapBackward(drDistorted);

    for (auto& v: boundaries[0])
        if (v[1] < tlUndistorted[1])
            tlUndistorted[1] = v[1];
    for (auto& v: boundaries[2])
        if (v[1] > drUndistorted[1])
            drUndistorted[1] = v[1];
    for (auto& v: boundaries[1])
        if (v[0] < tlUndistorted[0])
            tlUndistorted[0] = v[0];
    for (auto& v: boundaries[3])
        if (v[0] > drUndistorted[0])
            drUndistorted[0] = v[0];
}

void LensDistortionModelParameters::getRectMap(const Vector2dd &tl, const Vector2dd &dr, std::vector<Vector2dd> boundaries[4]) const
{
    Vector2dd shifts[] =
    {
        Vector2dd(1, 0),
        Vector2dd(0, 1),
        Vector2dd(1, 0),
        Vector2dd(0, 1)
    };
    Vector2dd origins[] =
    {
        tl,
        tl,
        Vector2dd(tl[0], dr[1]),
        Vector2dd(dr[0], tl[1])
    };
    int steps[] =
    {
        (int)std::ceil(dr[0] - tl[0]),
        (int)std::ceil(dr[1] - tl[1]),
        (int)std::ceil(dr[0] - tl[0]),
        (int)std::ceil(dr[1] - tl[1])
    };

    for (int i = 0; i < 4; ++i)
    {
        boundaries[i].resize(steps[i]);
        auto& boundary = boundaries[i];
        auto& shift    = shifts[i];
        auto& origin   = origins[i];
        auto& step     = steps[i];
        for (int j = 0; j < step; ++j)
            boundary[j] = map(origin + j * shift);
    }
}