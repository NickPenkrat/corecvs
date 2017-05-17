/**
 * \file inputFilterParameters.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "inputFilterParameters.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace corecvs {
template<>
Reflection BaseReflection<InputFilterParameters>::reflection = Reflection();
template<>
int BaseReflection<InputFilterParameters>::dummy = InputFilterParameters::staticInit();
} // namespace corecvs 

SUPPRESS_OFFSET_WARNING_BEGIN


using namespace corecvs;

int InputFilterParameters::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "Input Filter Parameters",
        "Input Filter Parameters",
        ""
    );

     getReflection()->objectSize = sizeof(InputFilterParameters);
     

    EnumField* field0 = new EnumField
        (
          InputFilterParameters::INPUT_TYPE_ID,
          offsetof(InputFilterParameters, mInputType),
          0,
          "Input Type",
          "Input Type",
          "Input Type",
          new EnumReflection(2
          , new EnumOption(0,"Left Frame")
          , new EnumOption(1,"Right Frame")
          )
        );
    field0->widgetHint=BaseField::COMBO_BOX;
    fields().push_back(field0);
    /*  */ 
    ReflectionDirectory &directory = *ReflectionDirectoryHolder::getReflectionDirectory();
    directory[std::string("Input Filter Parameters")]= &reflection;
   return 0;
}
int InputFilterParameters::relinkCompositeFields()
{
}

SUPPRESS_OFFSET_WARNING_END


