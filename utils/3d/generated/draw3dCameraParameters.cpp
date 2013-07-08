/**
 * \file draw3dCameraParameters.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "draw3dCameraParameters.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace core3vi {
template<>
Reflection BaseReflection<Draw3dCameraParameters>::reflection = Reflection();
template<>
int BaseReflection<Draw3dCameraParameters>::dummy = Draw3dCameraParameters::staticInit();
} // namespace core3vi 

SUPPRESS_OFFSET_WARNING_BEGIN

int Draw3dCameraParameters::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "draw 3d Camera Parameters",
        "draw 3d Camera Parameters",
        ""
    );
     

    fields().push_back(
        new DoubleField
        (
          Draw3dCameraParameters::FOVH_ID,
          offsetof(Draw3dCameraParameters, mFovH),
          90,
          "fovH",
          "fovH",
          "fovH"
        )
    );
    fields().push_back(
        new DoubleField
        (
          Draw3dCameraParameters::FOVV_ID,
          offsetof(Draw3dCameraParameters, mFovV),
          60,
          "fovV",
          "fovV",
          "fovV"
        )
    );
    fields().push_back(
        new DoubleField
        (
          Draw3dCameraParameters::NEARPLANE_ID,
          offsetof(Draw3dCameraParameters, mNearPlane),
          10,
          "nearPlane",
          "nearPlane",
          "nearPlane"
        )
    );
    fields().push_back(
        new DoubleField
        (
          Draw3dCameraParameters::FARPLANE_ID,
          offsetof(Draw3dCameraParameters, mFarPlane),
          100,
          "farPlane",
          "farPlane",
          "farPlane"
        )
    );
    fields().push_back(
        new EnumField
        (
          Draw3dCameraParameters::STYLE_ID,
          offsetof(Draw3dCameraParameters, mStyle),
          0,
          "style",
          "style",
          "style",
           NULL
        )
    );
    fields().push_back(
        new CompositeField
        (
          Draw3dCameraParameters::COLOR_ID,
          offsetof(Draw3dCameraParameters, mColor),
          "Color",
          "RgbColorParameters",
          "Color",
          "Color",
           NULL
        )
    );
    fields().push_back(
        new CompositeField
        (
          Draw3dCameraParameters::SECONDARY_COLOR_ID,
          offsetof(Draw3dCameraParameters, mSecondaryColor),
          "Secondary Color",
          "RgbColorParameters",
          "Secondary Color",
          "Secondary Color",
           NULL
        )
    );
    fields().push_back(
        new EnumField
        (
          Draw3dCameraParameters::TEXTURE_CORRODINATES_ID,
          offsetof(Draw3dCameraParameters, mTextureCorrodinates),
          0,
          "Texture Corrodinates",
          "Texture Corrodinates",
          "Texture Corrodinates",
           NULL
        )
    );
    fields().push_back(
        new IntField
        (
          Draw3dCameraParameters::TEXTURE_ALPHA_ID,
          offsetof(Draw3dCameraParameters, mTextureAlpha),
          255,
          "Texture Alpha",
          "Texture Alpha",
          "Texture Alpha"
        )
    );
    fields().push_back(
        new DoubleField
        (
          Draw3dCameraParameters::TEXTURE_SCALE_ID,
          offsetof(Draw3dCameraParameters, mTextureScale),
          1,
          "Texture Scale",
          "Texture Scale",
          "Texture Scale"
        )
    );
    fields().push_back(
        new IntField
        (
          Draw3dCameraParameters::DECAL_MATRIX_TYPE_ID,
          offsetof(Draw3dCameraParameters, mDecalMatrixType),
          11,
          "Decal Matrix Type",
          "Decal Matrix Type",
          "Decal Matrix Type"
        )
    );
    fields().push_back(
        new BoolField
        (
          Draw3dCameraParameters::DECAL_LEFT_CAM_ID,
          offsetof(Draw3dCameraParameters, mDecalLeftCam),
          false,
          "Decal Left Cam",
          "Decal Left Cam",
          "Decal Left Cam"
        )
    );
    fields().push_back(
        new IntField
        (
          Draw3dCameraParameters::DECAL_LEFT_ALPHA_ID,
          offsetof(Draw3dCameraParameters, mDecalLeftAlpha),
          255,
          "Decal Left Alpha",
          "Decal Left Alpha",
          "Decal Left Alpha"
        )
    );
    fields().push_back(
        new BoolField
        (
          Draw3dCameraParameters::DECAL_RIGHT_CAM_ID,
          offsetof(Draw3dCameraParameters, mDecalRightCam),
          false,
          "Decal Right Cam",
          "Decal Right Cam",
          "Decal Right Cam"
        )
    );
    fields().push_back(
        new IntField
        (
          Draw3dCameraParameters::DECAL_RIGHT_ALPHA_ID,
          offsetof(Draw3dCameraParameters, mDecalRightAlpha),
          255,
          "Decal Right Alpha",
          "Decal Right Alpha",
          "Decal Right Alpha"
        )
    );
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


