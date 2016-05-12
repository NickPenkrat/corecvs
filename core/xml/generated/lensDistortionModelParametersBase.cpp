/**
 * \file lensDistortionModelParametersBase.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "lensDistortionModelParametersBase.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace corecvs {
template<>
Reflection BaseReflection<LensDistortionModelParametersBase>::reflection = Reflection();
template<>
int BaseReflection<LensDistortionModelParametersBase>::dummy = LensDistortionModelParametersBase::staticInit();
} // namespace corecvs 

SUPPRESS_OFFSET_WARNING_BEGIN

int LensDistortionModelParametersBase::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "Lens Distortion Model Parameters Base",
        "Lens Distortion Model Parameters",
        ""
    );
     

    fields().push_back(
        new DoubleField
        (
          LensDistortionModelParametersBase::PRINCIPALX_ID,
          offsetof(LensDistortionModelParametersBase, mPrincipalX),
          240,
          "principalX",
          "principalX",
          "The center of the distortion \f$x_c\f$"
        )
    );
    fields().push_back(
        new DoubleField
        (
          LensDistortionModelParametersBase::PRINCIPALY_ID,
          offsetof(LensDistortionModelParametersBase, mPrincipalY),
          320,
          "principalY",
          "principalY",
          "The center of the distortion \f$y_c\f$"
        )
    );
    fields().push_back(
        new DoubleField
        (
          LensDistortionModelParametersBase::TANGENTIALX_ID,
          offsetof(LensDistortionModelParametersBase, mTangentialX),
          0,
          "tangentialX",
          "tangentialX",
          "First tangent correction coefficient - \f$p_1\f$"
        )
    );
    fields().push_back(
        new DoubleField
        (
          LensDistortionModelParametersBase::TANGENTIALY_ID,
          offsetof(LensDistortionModelParametersBase, mTangentialY),
          0,
          "tangentialY",
          "tangentialY",
          "Second tangent correction coefficient - \f$p_2\f$"
        )
    );
    double mKoeff_dv[] = {0,0,0,0,0,0};
    fields().push_back(
        new DoubleVectorField
        (
          LensDistortionModelParametersBase::KOEFF_ID,
          offsetof(LensDistortionModelParametersBase, mKoeff),
          vector<double>(mKoeff_dv, mKoeff_dv + 6),
          6,
          "koeff",
          "koeff",
          "Polynom to describe radial correction"
        )
    );
    fields().push_back(
        new DoubleField
        (
          LensDistortionModelParametersBase::ASPECT_ID,
          offsetof(LensDistortionModelParametersBase, mAspect),
          1,
          "aspect",
          "aspect",
          "aspect"
        )
    );
    fields().push_back(
        new DoubleField
        (
          LensDistortionModelParametersBase::SCALE_ID,
          offsetof(LensDistortionModelParametersBase, mScale),
          1,
          "scale",
          "scale",
          "scale"
        )
    );
    fields().push_back(
        new DoubleField
        (
          LensDistortionModelParametersBase::NORMALIZING_FOCAL_ID,
          offsetof(LensDistortionModelParametersBase, mNormalizingFocal),
          1,
          "Normalizing Focal",
          "Normalizing Focal",
          "Normalizing Focal"
        )
    );
    fields().push_back(
        new DoubleField
        (
          LensDistortionModelParametersBase::SHIFT_X_ID,
          offsetof(LensDistortionModelParametersBase, mShiftX),
          0,
          "shift X",
          "shift X",
          "Additional shift \f$x_s\f$"
        )
    );
    fields().push_back(
        new DoubleField
        (
          LensDistortionModelParametersBase::SHIFT_Y_ID,
          offsetof(LensDistortionModelParametersBase, mShiftY),
          0,
          "shift Y",
          "shift Y",
          "Additional shift \f$y_s\f$"
        )
    );
    fields().push_back(
        new BoolField
        (
          LensDistortionModelParametersBase::MAP_FORWARD_ID,
          offsetof(LensDistortionModelParametersBase, mMapForward),
          false,
          "Map Forward",
          "Map Forward",
          "This one is used to identify direction of map"
        )
    );
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


