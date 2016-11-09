/**
 * \file homorgaphyReconstructorBlockBase.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "homorgaphyReconstructorBlockBase.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace corecvs {
template<>
Reflection BaseReflection<HomorgaphyReconstructorBlockBase>::reflection = Reflection();
template<>
int BaseReflection<HomorgaphyReconstructorBlockBase>::dummy = HomorgaphyReconstructorBlockBase::staticInit();
} // namespace corecvs 

SUPPRESS_OFFSET_WARNING_BEGIN

int HomorgaphyReconstructorBlockBase::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "HomorgaphyReconstructorBlockBase",
        "HomorgaphyReconstructorBlockBase",
        ""
    );

     getReflection()->objectSize = sizeof(HomorgaphyReconstructorBlockBase);
     

    PointerField* field0 = new PointerField
        (
          HomorgaphyReconstructorBlockBase::IN0_ID,
          offsetof(HomorgaphyReconstructorBlockBase, mIn0),
          NULL,
          "in0",
          "in0",
          "in0",
          "CorrespondenceList"
        );
    fields().push_back(field0);
    /*  */ 
    PointerField* field1 = new PointerField
        (
          HomorgaphyReconstructorBlockBase::OUT0_ID,
          offsetof(HomorgaphyReconstructorBlockBase, mOut0),
          NULL,
          "out0",
          "out0",
          "out0",
          "Matrix33"
        );
    fields().push_back(field1);
    /*  */ 
    EnumField* field2 = new EnumField
        (
          HomorgaphyReconstructorBlockBase::ALGORITHM_ID,
          offsetof(HomorgaphyReconstructorBlockBase, mAlgorithm),
          4,
          "algorithm",
          "algorithm",
          "algorithm",
          new EnumReflection(5
          , new EnumOption(0,"LSE")
          , new EnumOption(1,"LSE1")
          , new EnumOption(2,"LSE2")
          , new EnumOption(3,"ML")
          , new EnumOption(4,"ML_AFTER_LSE")
          )
        );
    field2->widgetHint=BaseField::COMBO_BOX;
    fields().push_back(field2);
    /*  */ 
    ReflectionDirectory &directory = *ReflectionDirectoryHolder::getReflectionDirectory();
    directory[std::string("HomorgaphyReconstructorBlockBase")]= &reflection;
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


