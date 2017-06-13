/**
 * \file calibrationDrawHelpersParameters.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "calibrationDrawHelpersParameters.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace corecvs {
template<>
Reflection BaseReflection<CalibrationDrawHelpersParameters>::reflection = Reflection();
template<>
int BaseReflection<CalibrationDrawHelpersParameters>::dummy = CalibrationDrawHelpersParameters::staticInit();
} // namespace corecvs 

SUPPRESS_OFFSET_WARNING_BEGIN


using namespace corecvs;

int CalibrationDrawHelpersParameters::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "Calibration Draw Helpers Parameters",
        "Calibration Draw Helpers Parameters",
        ""
    );

     getReflection()->objectSize = sizeof(CalibrationDrawHelpersParameters);
     

    BoolField* field0 = new BoolField
        (
          CalibrationDrawHelpersParameters::USE_OLD_BACKEND_ID,
          offsetof(CalibrationDrawHelpersParameters, mUseOldBackend),
          true,
          "Use Old Backend",
          "Use Old Backend",
          "We have two OpenGL backends to draw. Old is without shaders, new is with shaders"
        );
    field0->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field0);
    /*  */ 
    DoubleField* field1 = new DoubleField
        (
          CalibrationDrawHelpersParameters::SCALE_FOR_CAMERAS_ID,
          offsetof(CalibrationDrawHelpersParameters, mScaleForCameras),
          1,
          "Scale For Cameras",
          "Scale For Cameras",
          "Scale For Cameras",
          true,
         0.01,
         1000
        );
    field1->widgetHint=BaseField::SPIN_BOX;
    field1->precision=2;
    fields().push_back(field1);
    /*  */ 
    BoolField* field2 = new BoolField
        (
          CalibrationDrawHelpersParameters::PRINTNAMES_ID,
          offsetof(CalibrationDrawHelpersParameters, mPrintNames),
          false,
          "printNames",
          "printNames",
          "printNames"
        );
    field2->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field2);
    /*  */ 
    BoolField* field3 = new BoolField
        (
          CalibrationDrawHelpersParameters::BILLBOARDNAMES_ID,
          offsetof(CalibrationDrawHelpersParameters, mBillboardNames),
          true,
          "billboardNames",
          "billboardNames",
          "billboardNames"
        );
    field3->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field3);
    /*  */ 
    BoolField* field4 = new BoolField
        (
          CalibrationDrawHelpersParameters::PREFER_REPROJECTED_ID,
          offsetof(CalibrationDrawHelpersParameters, mPreferReprojected),
          true,
          "Prefer Reprojected",
          "Prefer Reprojected",
          "Prefer Reprojected"
        );
    field4->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field4);
    /*  */ 
    BoolField* field5 = new BoolField
        (
          CalibrationDrawHelpersParameters::FORCE_KNOWN_ID,
          offsetof(CalibrationDrawHelpersParameters, mForceKnown),
          true,
          "Force Known",
          "Force Known",
          "Force Known"
        );
    field5->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field5);
    /*  */ 
    BoolField* field6 = new BoolField
        (
          CalibrationDrawHelpersParameters::PRIVATECOLOR_ID,
          offsetof(CalibrationDrawHelpersParameters, mPrivateColor),
          false,
          "privateColor",
          "privateColor",
          "privateColor"
        );
    field6->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field6);
    /*  */ 
    BoolField* field7 = new BoolField
        (
          CalibrationDrawHelpersParameters::LARGEPOINTS_ID,
          offsetof(CalibrationDrawHelpersParameters, mLargePoints),
          false,
          "largePoints",
          "largePoints",
          "largePoints"
        );
    field7->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field7);
    /*  */ 
    BoolField* field8 = new BoolField
        (
          CalibrationDrawHelpersParameters::DRAWFIXTURECAMS_ID,
          offsetof(CalibrationDrawHelpersParameters, mDrawFixtureCams),
          true,
          "drawFixtureCams",
          "drawFixtureCams",
          "drawFixtureCams"
        );
    field8->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field8);
    /*  */ 
    BoolField* field9 = new BoolField
        (
          CalibrationDrawHelpersParameters::DRAWOBSERVATIONS_ID,
          offsetof(CalibrationDrawHelpersParameters, mDrawObservations),
          false,
          "drawObservations",
          "drawObservations",
          "drawObservations"
        );
    field9->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field9);
    /*  */ 
    BoolField* field10 = new BoolField
        (
          CalibrationDrawHelpersParameters::DRAWTRUELINES_ID,
          offsetof(CalibrationDrawHelpersParameters, mDrawTrueLines),
          false,
          "drawTrueLines",
          "drawTrueLines",
          "drawTrueLines"
        );
    field10->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field10);
    /*  */ 
    DoubleField* field11 = new DoubleField
        (
          CalibrationDrawHelpersParameters::PROJECTION_RAY_LENGTH_ID,
          offsetof(CalibrationDrawHelpersParameters, mProjectionRayLength),
          10,
          "Projection Ray Length",
          "Projection Ray Length",
          "Projection Ray Length",
          true,
         0.01,
         1000
        );
    field11->widgetHint=BaseField::SPIN_BOX;
    field11->precision=2;
    fields().push_back(field11);
    /*  */ 
    BoolField* field12 = new BoolField
        (
          CalibrationDrawHelpersParameters::DRAWRAYS_ID,
          offsetof(CalibrationDrawHelpersParameters, mDrawRays),
          true,
          "drawRays",
          "drawRays",
          "drawRays"
        );
    field12->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field12);
    /*  */ 
    ReflectionDirectory &directory = *ReflectionDirectoryHolder::getReflectionDirectory();
    directory[std::string("Calibration Draw Helpers Parameters")]= &reflection;
   return 0;
}
int CalibrationDrawHelpersParameters::relinkCompositeFields()
{
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


