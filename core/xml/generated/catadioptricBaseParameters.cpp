/**
 * \file catadioptricBaseParameters.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "catadioptricBaseParameters.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace corecvs {
template<>
Reflection BaseReflection<CatadioptricBaseParameters>::reflection = Reflection();
template<>
int BaseReflection<CatadioptricBaseParameters>::dummy = CatadioptricBaseParameters::staticInit();
} // namespace corecvs 

SUPPRESS_OFFSET_WARNING_BEGIN


using namespace corecvs;

int CatadioptricBaseParameters::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "Catadioptric Base Parameters",
        "Catadioptric Base Parameters",
        ""
    );

     getReflection()->objectSize = sizeof(CatadioptricBaseParameters);
     

    DoubleField* field0 = new DoubleField
        (
          CatadioptricBaseParameters::PRINCIPALX_ID,
          offsetof(CatadioptricBaseParameters, mPrincipalX),
          240,
          "principalX",
          "principalX",
          "The center of projection \f$x_c\f$",
          true,
         0,
         99999
        );
    field0->widgetHint=BaseField::SPIN_BOX;
    field0->precision=2;
    fields().push_back(field0);
    /*  */ 
    DoubleField* field1 = new DoubleField
        (
          CatadioptricBaseParameters::PRINCIPALY_ID,
          offsetof(CatadioptricBaseParameters, mPrincipalY),
          320,
          "principalY",
          "principalY",
          "The center of projection \f$y_c\f$",
          true,
         0,
         99999
        );
    field1->widgetHint=BaseField::SPIN_BOX;
    field1->precision=2;
    fields().push_back(field1);
    /*  */ 
    DoubleField* field2 = new DoubleField
        (
          CatadioptricBaseParameters::FOCAL_ID,
          offsetof(CatadioptricBaseParameters, mFocal),
          820.427,
          "focal",
          "focal",
          "focal",
          true,
         0,
         99999
        );
    field2->widgetHint=BaseField::SPIN_BOX;
    field2->precision=2;
    fields().push_back(field2);
    /*  */ 
    double mR_dv[] = {0,0,0,0,0,0};
    DoubleVectorField* field3 = new DoubleVectorField
        (
          CatadioptricBaseParameters::R_ID,
          offsetof(CatadioptricBaseParameters, mR),
          vector<double>(mR_dv, mR_dv + 6),
          6,
          "r",
          "r",
          "r"
        );
    fields().push_back(field3);
    /*  */ 
    DoubleField* field4 = new DoubleField
        (
          CatadioptricBaseParameters::SIZEX_ID,
          offsetof(CatadioptricBaseParameters, mSizeX),
          240,
          "sizeX",
          "sizeX",
          "Model image resolution X",
          true,
         0,
         99999
        );
    field4->widgetHint=BaseField::SPIN_BOX;
    field4->suffixHint="px";
    field4->precision=2;
    fields().push_back(field4);
    /*  */ 
    DoubleField* field5 = new DoubleField
        (
          CatadioptricBaseParameters::SIZEY_ID,
          offsetof(CatadioptricBaseParameters, mSizeY),
          320,
          "sizeY",
          "sizeY",
          "Model image resolution Y",
          true,
         0,
         99999
        );
    field5->widgetHint=BaseField::SPIN_BOX;
    field5->suffixHint="px";
    field5->precision=2;
    fields().push_back(field5);
    /*  */ 
    DoubleField* field6 = new DoubleField
        (
          CatadioptricBaseParameters::DISTORTEDSIZEX_ID,
          offsetof(CatadioptricBaseParameters, mDistortedSizeX),
          240,
          "distortedSizeX",
          "distortedSizeX",
          "Source image resolution X",
          true,
         0,
         99999
        );
    field6->widgetHint=BaseField::SPIN_BOX;
    field6->precision=2;
    fields().push_back(field6);
    /*  */ 
    DoubleField* field7 = new DoubleField
        (
          CatadioptricBaseParameters::DISTORTEDSIZEY_ID,
          offsetof(CatadioptricBaseParameters, mDistortedSizeY),
          320,
          "distortedSizeY",
          "distortedSizeY",
          "Source image resolution Y",
          true,
         0,
         99999
        );
    field7->widgetHint=BaseField::SPIN_BOX;
    field7->precision=2;
    fields().push_back(field7);
    /*  */ 
    ReflectionDirectory &directory = *ReflectionDirectoryHolder::getReflectionDirectory();
    directory[std::string("Catadioptric Base Parameters")]= &reflection;
   return 0;
}
int CatadioptricBaseParameters::relinkCompositeFields()
{
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


