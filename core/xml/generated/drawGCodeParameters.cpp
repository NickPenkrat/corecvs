/**
 * \file drawGCodeParameters.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "drawGCodeParameters.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace corecvs {
template<>
Reflection BaseReflection<DrawGCodeParameters>::reflection = Reflection();
template<>
int BaseReflection<DrawGCodeParameters>::dummy = DrawGCodeParameters::staticInit();
} // namespace corecvs 

SUPPRESS_OFFSET_WARNING_BEGIN


using namespace corecvs;

int DrawGCodeParameters::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "draw GCode Parameters",
        "draw GCode Parameters",
        ""
    );

     getReflection()->objectSize = sizeof(DrawGCodeParameters);
     

    EnumField* field0 = new EnumField
        (
          DrawGCodeParameters::SCHEME_ID,
          offsetof(DrawGCodeParameters, mScheme),
          0,
          "scheme",
          "scheme",
          "scheme",
          new EnumReflection(4
          , new EnumOption(0,"Color From GCode")
          , new EnumOption(1,"Color From Extrusion Rate")
          , new EnumOption(2,"Color From Temperature")
          , new EnumOption(3,"Color From Speed")
          )
        );
    field0->widgetHint=BaseField::COMBO_BOX;
    fields().push_back(field0);
    /*  */ 
    DoubleField* field1 = new DoubleField
        (
          DrawGCodeParameters::MINTEMP_ID,
          offsetof(DrawGCodeParameters, mMinTemp),
          200,
          "minTemp",
          "minTemp",
          "minTemp",
          true,
         0,
         1000
        );
    field1->widgetHint=BaseField::SPIN_BOX;
    field1->precision=2;
    fields().push_back(field1);
    /*  */ 
    DoubleField* field2 = new DoubleField
        (
          DrawGCodeParameters::MAXTEMP_ID,
          offsetof(DrawGCodeParameters, mMaxTemp),
          250,
          "maxTemp",
          "maxTemp",
          "maxTemp",
          true,
         0,
         1000
        );
    field2->widgetHint=BaseField::SPIN_BOX;
    field2->precision=2;
    fields().push_back(field2);
    /*  */ 
    DoubleField* field3 = new DoubleField
        (
          DrawGCodeParameters::MINSPEED_ID,
          offsetof(DrawGCodeParameters, mMinSpeed),
          200,
          "minSpeed",
          "minSpeed",
          "minSpeed",
          true,
         0,
         1000
        );
    field3->widgetHint=BaseField::SPIN_BOX;
    field3->precision=2;
    fields().push_back(field3);
    /*  */ 
    DoubleField* field4 = new DoubleField
        (
          DrawGCodeParameters::MAXSPEED_ID,
          offsetof(DrawGCodeParameters, mMaxSpeed),
          950,
          "maxSpeed",
          "maxSpeed",
          "maxSpeed",
          true,
         0,
         1000
        );
    field4->widgetHint=BaseField::SPIN_BOX;
    field4->precision=2;
    fields().push_back(field4);
    /*  */ 
    ReflectionDirectory &directory = *ReflectionDirectoryHolder::getReflectionDirectory();
    directory[std::string("draw GCode Parameters")]= &reflection;
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


