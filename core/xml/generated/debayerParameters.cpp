/**
 * \file debayerParameters.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "debayerParameters.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace corecvs {
template<>
Reflection BaseReflection<DebayerParameters>::reflection = Reflection();
template<>
int BaseReflection<DebayerParameters>::dummy = DebayerParameters::staticInit();
} // namespace corecvs 

SUPPRESS_OFFSET_WARNING_BEGIN


using namespace corecvs;

int DebayerParameters::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "Debayer Parameters",
        "Debayer Parameters",
        ""
    );

     getReflection()->objectSize = sizeof(DebayerParameters);
     

    EnumField* field0 = new EnumField
        (
          DebayerParameters::METHOD_ID,
          offsetof(DebayerParameters, mMethod),
          0,
          "Method",
          "Method",
          "Method",
          new EnumReflection(4
          , new EnumOption(0,"Nearest")
          , new EnumOption(1,"Bilinear")
          , new EnumOption(2,"AHD")
          , new EnumOption(3,"Fourier")
          )
        );
    field0->widgetHint=BaseField::COMBO_BOX;
    fields().push_back(field0);
    /*  */ 
    IntField* field1 = new IntField
        (
          DebayerParameters::BAYER_POS_ID,
          offsetof(DebayerParameters, mBayerPos),
          -1,
          "Bayer pos",
          "Bayer pos",
          "Bayer pos",
          true,
         -1,
         255
        );
    fields().push_back(field1);
    /*  */ 
    ReflectionDirectory &directory = *ReflectionDirectoryHolder::getReflectionDirectory();
    directory[std::string("Debayer Parameters")]= &reflection;
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


