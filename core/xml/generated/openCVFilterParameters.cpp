/**
 * \file openCVFilterParameters.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "openCVFilterParameters.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace core3vi {
template<>
Reflection BaseReflection<OpenCVFilterParameters>::reflection = Reflection();
template<>
int BaseReflection<OpenCVFilterParameters>::dummy = OpenCVFilterParameters::staticInit();
} // namespace core3vi 

SUPPRESS_OFFSET_WARNING_BEGIN

int OpenCVFilterParameters::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "OpenCV Filter Parameters",
        "OpenCV Filter Parameters",
        ""
    );
     

    fields().push_back(
        new EnumField
        (
          OpenCVFilterParameters::OPENCVFILTER_ID,
          offsetof(OpenCVFilterParameters, mOpenCVFilter),
          0,
          "OpenCVFilter",
          "OpenCVFilter",
          "OpenCVFilter",
           NULL
        )
    );
    fields().push_back(
        new IntField
        (
          OpenCVFilterParameters::PARAM1_ID,
          offsetof(OpenCVFilterParameters, mParam1),
          50,
          "Param1",
          "Param1",
          "Param1"
        )
    );
    fields().push_back(
        new IntField
        (
          OpenCVFilterParameters::PARAM2_ID,
          offsetof(OpenCVFilterParameters, mParam2),
          100,
          "Param2",
          "Param2",
          "Param2"
        )
    );
   return 0;
}

SUPPRESS_OFFSET_WARNING_END

