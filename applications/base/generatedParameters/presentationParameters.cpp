/**
 * \file presentationParameters.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "presentationParameters.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace corecvs {
template<>
Reflection BaseReflection<PresentationParameters>::reflection = Reflection();
template<>
int BaseReflection<PresentationParameters>::dummy = PresentationParameters::staticInit();
} // namespace corecvs 

SUPPRESS_OFFSET_WARNING_BEGIN

int PresentationParameters::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "Presentation Parameters",
        "Presentation parameters",
        ""
    );
     

    EnumField* field0 = new EnumField
        (
          PresentationParameters::OUTPUT_ID,
          offsetof(PresentationParameters, mOutput),
          0,
          "Output",
          "Output",
          "View - views are more or less self-explenatory",
          new EnumReflection(7
          , new EnumOption(0,"right Frame")
          , new EnumOption(1,"left Frame")
          , new EnumOption(2,"sidebyside stereo")
          , new EnumOption(3,"anaglyph RG")
          , new EnumOption(4,"anaglyph RC")
          , new EnumOption(5,"blend")
          , new EnumOption(6,"none")
          )
        );
    field0->widgetHint=BaseField::COMBO_BOX;
    field0->precision=-1;
    fields().push_back(field0);
    /*  */ 
    EnumField* field1 = new EnumField
        (
          PresentationParameters::STEREO_ID,
          offsetof(PresentationParameters, mStereo),
          0,
          "Stereo",
          "Stereo",
          "Way to draw overlay with disparity information",
          new EnumReflection(6
          , new EnumOption(0,"dont show stereo")
          , new EnumOption(1,"fast large dots")
          , new EnumOption(2,"fast dots")
          , new EnumOption(3,"show lines stereo")
          , new EnumOption(4,"show all stereo")
          , new EnumOption(5,"show dots stereo")
          )
        );
    field1->widgetHint=BaseField::COMBO_BOX;
    field1->precision=-1;
    fields().push_back(field1);
    /*  */ 
    EnumField* field2 = new EnumField
        (
          PresentationParameters::FLOW_ID,
          offsetof(PresentationParameters, mFlow),
          0,
          "Flow",
          "Flow",
          "Way to draw overlay with optical flow information",
          new EnumReflection(6
          , new EnumOption(0,"dont show flow")
          , new EnumOption(1,"fast colored dots")
          , new EnumOption(2,"show lines only")
          , new EnumOption(3,"show colored dots")
          , new EnumOption(4,"show colored lines")
          , new EnumOption(5,"show heat coloring")
          )
        );
    field2->widgetHint=BaseField::COMBO_BOX;
    field2->precision=-1;
    fields().push_back(field2);
    /*  */ 
    BoolField* field3 = new BoolField
        (
          PresentationParameters::SHOWCLUSTERS_ID,
          offsetof(PresentationParameters, mShowClusters),
          false,
          "showClusters",
          "showClusters",
          "showClusters"
        );
    field3->widgetHint=BaseField::CHECK_BOX;
    field3->precision=-1;
    fields().push_back(field3);
    /*  */ 
    BoolField* field4 = new BoolField
        (
          PresentationParameters::SHOWHISTOGRAM_ID,
          offsetof(PresentationParameters, mShowHistogram),
          true,
          "showHistogram",
          "showHistogram",
          "showHistogram"
        );
    field4->widgetHint=BaseField::CHECK_BOX;
    field4->precision=-1;
    fields().push_back(field4);
    /*  */ 
    BoolField* field5 = new BoolField
        (
          PresentationParameters::AUTO_UPDATE_HISTOGRAM_ID,
          offsetof(PresentationParameters, mAutoUpdateHistogram),
          false,
          "Auto Update Histogram",
          "Auto Update Histogram",
          "Auto Update Histogram"
        );
    field5->widgetHint=BaseField::CHECK_BOX;
    field5->precision=-1;
    fields().push_back(field5);
    /*  */ 
    BoolField* field6 = new BoolField
        (
          PresentationParameters::SHOWAREAOFINTEREST_ID,
          offsetof(PresentationParameters, mShowAreaOfInterest),
          true,
          "showAreaOfInterest",
          "showAreaOfInterest",
          "showAreaOfInterest"
        );
    field6->widgetHint=BaseField::CHECK_BOX;
    field6->precision=-1;
    fields().push_back(field6);
    /*  */ 
    BoolField* field7 = new BoolField
        (
          PresentationParameters::PRODUCE3D_ID,
          offsetof(PresentationParameters, mProduce3D),
          true,
          "produce3D",
          "produce3D",
          "produce3D"
        );
    field7->widgetHint=BaseField::CHECK_BOX;
    field7->precision=-1;
    fields().push_back(field7);
    /*  */ 
    BoolField* field8 = new BoolField
        (
          PresentationParameters::PRODUCE6D_ID,
          offsetof(PresentationParameters, mProduce6D),
          false,
          "produce6D",
          "produce6D",
          "produce6D"
        );
    field8->widgetHint=BaseField::CHECK_BOX;
    field8->precision=-1;
    fields().push_back(field8);
    /*  */ 
    BoolField* field9 = new BoolField
        (
          PresentationParameters::DUMP3D_ID,
          offsetof(PresentationParameters, mDump3D),
          false,
          "dump3D",
          "dump3D",
          "dump3D"
        );
    field9->widgetHint=BaseField::CHECK_BOX;
    field9->precision=-1;
    fields().push_back(field9);
    /*  */ 
    ReflectionDirectory &directory = *ReflectionDirectoryHolder::getReflectionDirectory();
    directory[std::string("Presentation Parameters")]= &reflection;
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


