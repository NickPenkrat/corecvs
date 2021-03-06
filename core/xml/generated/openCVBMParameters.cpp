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


using namespace corecvs;

int OpenCVBMParameters::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "OpenCV BM Parameters",
        "OpenCV BM Parameters Class",
        ""
    );

     getReflection()->objectSize = sizeof(OpenCVBMParameters);
     

    IntField* field0 = new IntField
        (
          OpenCVBMParameters::BLOCK_SIZE_ID,
          offsetof(OpenCVBMParameters, mBlockSize),
          21,
          "Block Size",
          "Block Size",
          "Block Size",
          true,
         5,
         255,
         2
        );
    fields().push_back(field0);
    /*  */ 
    IntField* field1 = new IntField
        (
          OpenCVBMParameters::DISPARITY_SEARCH_ID,
          offsetof(OpenCVBMParameters, mDisparitySearch),
          0,
          "Disparity search",
          "Disparity search",
          "Disparity search",
          true,
         0,
         1600,
         16
        );
    fields().push_back(field1);
    /*  */ 
    IntField* field2 = new IntField
        (
          OpenCVBMParameters::PREFILTERCAP_ID,
          offsetof(OpenCVBMParameters, mPreFilterCap),
          31,
          "preFilterCap",
          "preFilterCap",
          "preFilterCap",
          true,
         1,
         1000,
         1
        );
    fields().push_back(field2);
    /*  */ 
    IntField* field3 = new IntField
        (
          OpenCVBMParameters::MINDISPARITY_ID,
          offsetof(OpenCVBMParameters, mMinDisparity),
          1,
          "minDisparity",
          "minDisparity",
          "minDisparity",
          true,
         1,
         1000,
         1
        );
    fields().push_back(field3);
    /*  */ 
    IntField* field4 = new IntField
        (
          OpenCVBMParameters::TEXTURETHRESHOLD_ID,
          offsetof(OpenCVBMParameters, mTextureThreshold),
          10,
          "textureThreshold",
          "textureThreshold",
          "textureThreshold",
          true,
         0,
         1000,
         1
        );
    fields().push_back(field4);
    /*  */ 
    IntField* field5 = new IntField
        (
          OpenCVBMParameters::UNIQUENESSRATIO_ID,
          offsetof(OpenCVBMParameters, mUniquenessRatio),
          15,
          "uniquenessRatio",
          "uniquenessRatio",
          "uniquenessRatio",
          true,
         0,
         1000,
         1
        );
    fields().push_back(field5);
    /*  */ 
    IntField* field6 = new IntField
        (
          OpenCVBMParameters::SPECKLEWINDOWSIZE_ID,
          offsetof(OpenCVBMParameters, mSpeckleWindowSize),
          100,
          "speckleWindowSize",
          "speckleWindowSize",
          "speckleWindowSize",
          true,
         0,
         1000,
         1
        );
    fields().push_back(field6);
    /*  */ 
    IntField* field7 = new IntField
        (
          OpenCVBMParameters::SPECKLERANGE_ID,
          offsetof(OpenCVBMParameters, mSpeckleRange),
          32,
          "speckleRange",
          "speckleRange",
          "speckleRange",
          true,
         0,
         1000,
         1
        );
    fields().push_back(field7);
    /*  */ 
    IntField* field8 = new IntField
        (
          OpenCVBMParameters::DISP12MAXDIFF_ID,
          offsetof(OpenCVBMParameters, mDisp12MaxDiff),
          1,
          "disp12MaxDiff",
          "disp12MaxDiff",
          "disp12MaxDiff",
          true,
         -1,
         1000,
         1
        );
    fields().push_back(field8);
    /*  */ 
    ReflectionDirectory &directory = *ReflectionDirectoryHolder::getReflectionDirectory();
    directory[std::string("OpenCV BM Parameters")]= &reflection;
   return 0;
}
int OpenCVBMParameters::relinkCompositeFields()
{
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


