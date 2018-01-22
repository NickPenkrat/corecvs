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
          1,
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
         3
        );
    fields().push_back(field1);
    /*  */ 
    IntField* field2 = new IntField
        (
          DebayerParameters::NUMBITSOUT_ID,
          offsetof(DebayerParameters, mNumBitsOut),
          12,
          "numBitsOut",
          "numBitsOut",
          "numBitsOut",
          true,
         -1,
         15
        );
    fields().push_back(field2);
    /*  */ 
    double mGains_dv[] = {1,1,1};
    DoubleVectorField* field3 = new DoubleVectorField
        (
          DebayerParameters::GAINS_ID,
          offsetof(DebayerParameters, mGains),
          vector<double>(mGains_dv, mGains_dv + 3),
          3,
          "gains",
          "gains",
          "RGB gains"
        );
    fields().push_back(field3);
    /*  */ 
    double mGamma_dv[] = {1,1};
    DoubleVectorField* field4 = new DoubleVectorField
        (
          DebayerParameters::GAMMA_ID,
          offsetof(DebayerParameters, mGamma),
          vector<double>(mGamma_dv, mGamma_dv + 2),
          2,
          "gamma",
          "gamma",
          "Gamma values"
        );
    fields().push_back(field4);
    /*  */ 
    ReflectionDirectory &directory = *ReflectionDirectoryHolder::getReflectionDirectory();
    directory[std::string("Debayer Parameters")]= &reflection;
   return 0;
}
int DebayerParameters::relinkCompositeFields()
{
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


