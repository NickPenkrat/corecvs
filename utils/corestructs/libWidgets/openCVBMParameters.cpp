/**
 * \file openCVBMParameters.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "openCVBMParameters.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace corecvs {
template<>
Reflection BaseReflection<OpenCVBMParameters>::reflection = Reflection();
template<>
int BaseReflection<OpenCVBMParameters>::dummy = OpenCVBMParameters::staticInit();
} // namespace corecvs 

SUPPRESS_OFFSET_WARNING_BEGIN

int OpenCVBMParameters::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "OpenCV BM Parameters",
        "OpenCV BM Parameters Class",
        ""
    );
     

    fields().push_back(
        new IntField
        (
          OpenCVBMParameters::BLOCK_SIZE_ID,
          offsetof(OpenCVBMParameters, mBlockSize),
          21,
          "Block Size",
          "Block Size",
          "Block Size"
        )
    );
    fields().push_back(
        new IntField
        (
          OpenCVBMParameters::DISPARITY_SEARCH_ID,
          offsetof(OpenCVBMParameters, mDisparitySearch),
          0,
          "Disparity search",
          "Disparity search",
          "Disparity search"
        )
    );
    fields().push_back(
        new IntField
        (
          OpenCVBMParameters::PREFILTERCAP_ID,
          offsetof(OpenCVBMParameters, mPreFilterCap),
          31,
          "preFilterCap",
          "preFilterCap",
          "preFilterCap"
        )
    );
    fields().push_back(
        new IntField
        (
          OpenCVBMParameters::MINDISPARITY_ID,
          offsetof(OpenCVBMParameters, mMinDisparity),
          1,
          "minDisparity",
          "minDisparity",
          "minDisparity"
        )
    );
    fields().push_back(
        new IntField
        (
          OpenCVBMParameters::TEXTURETHRESHOLD_ID,
          offsetof(OpenCVBMParameters, mTextureThreshold),
          10,
          "textureThreshold",
          "textureThreshold",
          "textureThreshold"
        )
    );
    fields().push_back(
        new IntField
        (
          OpenCVBMParameters::UNIQUENESSRATIO_ID,
          offsetof(OpenCVBMParameters, mUniquenessRatio),
          15,
          "uniquenessRatio",
          "uniquenessRatio",
          "uniquenessRatio"
        )
    );
    fields().push_back(
        new IntField
        (
          OpenCVBMParameters::SPECKLEWINDOWSIZE_ID,
          offsetof(OpenCVBMParameters, mSpeckleWindowSize),
          100,
          "speckleWindowSize",
          "speckleWindowSize",
          "speckleWindowSize"
        )
    );
    fields().push_back(
        new IntField
        (
          OpenCVBMParameters::SPECKLERANGE_ID,
          offsetof(OpenCVBMParameters, mSpeckleRange),
          32,
          "speckleRange",
          "speckleRange",
          "speckleRange"
        )
    );
    fields().push_back(
        new IntField
        (
          OpenCVBMParameters::DISP12MAXDIFF_ID,
          offsetof(OpenCVBMParameters, mDisp12MaxDiff),
          1,
          "disp12MaxDiff",
          "disp12MaxDiff",
          "disp12MaxDiff"
        )
    );
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


