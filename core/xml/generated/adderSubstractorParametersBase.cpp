/**
 * \file adderSubstractorParametersBase.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "adderSubstractorParametersBase.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace corecvs {
template<>
Reflection BaseReflection<AdderSubstractorParametersBase>::reflection = Reflection();
template<>
int BaseReflection<AdderSubstractorParametersBase>::dummy = AdderSubstractorParametersBase::staticInit();
} // namespace corecvs 

SUPPRESS_OFFSET_WARNING_BEGIN


using namespace corecvs;

int AdderSubstractorParametersBase::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "Adder Substractor Parameters Base",
        "Adder Substractor Parameters Base",
        ""
    );

     getReflection()->objectSize = sizeof(AdderSubstractorParametersBase);
     

    DoubleField* field0 = new DoubleField
        (
          AdderSubstractorParametersBase::INPUT1_ID,
          offsetof(AdderSubstractorParametersBase, mInput1),
          0,
          "input1",
          "input1",
          "input1"
        );
    field0->widgetHint=BaseField::SPIN_BOX;
    field0->precision=2;
    fields().push_back(field0);
    /*  */ 
    DoubleField* field1 = new DoubleField
        (
          AdderSubstractorParametersBase::INPUT2_ID,
          offsetof(AdderSubstractorParametersBase, mInput2),
          0,
          "input2",
          "input2",
          "input2"
        );
    field1->widgetHint=BaseField::SPIN_BOX;
    field1->precision=2;
    fields().push_back(field1);
    /*  */ 
    BoolField* field2 = new BoolField
        (
          AdderSubstractorParametersBase::PARAMETER_ID,
          offsetof(AdderSubstractorParametersBase, mParameter),
          false,
          "parameter",
          "parameter",
          "parameter"
        );
    field2->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field2);
    /*  */ 
    DoubleField* field3 = new DoubleField
        (
          AdderSubstractorParametersBase::OUTPUT1_ID,
          offsetof(AdderSubstractorParametersBase, mOutput1),
          0,
          "output1",
          "output1",
          "output1"
        );
    field3->widgetHint=BaseField::SPIN_BOX;
    field3->precision=2;
    fields().push_back(field3);
    /*  */ 
    DoubleField* field4 = new DoubleField
        (
          AdderSubstractorParametersBase::OUTPUT2_ID,
          offsetof(AdderSubstractorParametersBase, mOutput2),
          0,
          "output2",
          "output2",
          "output2"
        );
    field4->widgetHint=BaseField::SPIN_BOX;
    field4->precision=2;
    fields().push_back(field4);
    /*  */ 
    ReflectionDirectory &directory = *ReflectionDirectoryHolder::getReflectionDirectory();
    directory[std::string("Adder Substractor Parameters Base")]= &reflection;
   return 0;
}
int AdderSubstractorParametersBase::relinkCompositeFields()
{
}

SUPPRESS_OFFSET_WARNING_END


