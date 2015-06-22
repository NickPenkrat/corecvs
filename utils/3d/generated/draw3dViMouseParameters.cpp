/**
 * \file draw3dViMouseParameters.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "draw3dViMouseParameters.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace corecvs {
template<>
Reflection BaseReflection<Draw3dViMouseParameters>::reflection = Reflection();
template<>
int BaseReflection<Draw3dViMouseParameters>::dummy = Draw3dViMouseParameters::staticInit();
} // namespace corecvs 

SUPPRESS_OFFSET_WARNING_BEGIN

int Draw3dViMouseParameters::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "draw 3d ViMouse Parameters",
        "draw 3d ViMouse Parameters",
        ""
    );
     

    fields().push_back(
        new DoubleField
        (
          Draw3dViMouseParameters::REDDIST_ID,
          offsetof(Draw3dViMouseParameters, mRedDist),
          0,
          "redDist",
          "redDist",
          "redDist",
          true,
         -50000,
         50000
        )
    );
    fields().push_back(
        new DoubleField
        (
          Draw3dViMouseParameters::BLUEDIST_ID,
          offsetof(Draw3dViMouseParameters, mBlueDist),
          1000,
          "blueDist",
          "blueDist",
          "blueDist",
          true,
         -50000,
         50000
        )
    );
    fields().push_back(
        new DoubleField
        (
          Draw3dViMouseParameters::FLOWZOOM_ID,
          offsetof(Draw3dViMouseParameters, mFlowZoom),
          1,
          "flowZoom",
          "flowZoom",
          "flowZoom",
          true,
         0,
         50
        )
    );
    fields().push_back(
        new EnumField
        (
          Draw3dViMouseParameters::POINT_COLOR_TYPE_ID,
          offsetof(Draw3dViMouseParameters, mPointColorType),
          0,
          "Point Color Type",
          "Point Color Type",
          "Point Color Type",
          new EnumReflection(7
          , new EnumOption(0,"Grey value")
          , new EnumOption(1,"Z Coordinate")
          , new EnumOption(2,"Y Coordinate")
          , new EnumOption(3,"Distance")
          , new EnumOption(4,"None")
          , new EnumOption(5,"By Flag")
          , new EnumOption(6,"By Cluster")
          )
        )
    );
    fields().push_back(
        new EnumField
        (
          Draw3dViMouseParameters::FLOW_COLOR_TYPE_ID,
          offsetof(Draw3dViMouseParameters, mFlowColorType),
          0,
          "Flow Color Type",
          "Flow Color Type",
          "Flow Color Type",
          new EnumReflection(4
          , new EnumOption(0,"None")
          , new EnumOption(1,"White")
          , new EnumOption(2,"Heat")
          , new EnumOption(3,"By Flag")
          )
        )
    );
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


