/**
 * \file merger.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "merger.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace corecvs {
template<>
Reflection BaseReflection<Merger>::reflection = Reflection();
template<>
int BaseReflection<Merger>::dummy = Merger::staticInit();
} // namespace corecvs 

SUPPRESS_OFFSET_WARNING_BEGIN


using namespace corecvs;

int Merger::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "Merger",
        "Merger parameters",
        ""
    );

     getReflection()->objectSize = sizeof(Merger);
     

    EnumField* field0 = new EnumField
        (
          Merger::UNDISTMETHOD_ID,
          offsetof(Merger, mUndistMethod),
          3,
          "undistMethod",
          "undistMethod",
          "undistMethod",
          new EnumReflection(5
          , new EnumOption(0,"None")
          , new EnumOption(1,"Square Table")
          , new EnumOption(2,"Radial Table")
          , new EnumOption(3,"HD Table")
          , new EnumOption(4,"Loaded Camera")
          )
        );
    field0->widgetHint=BaseField::COMBO_BOX;
    fields().push_back(field0);
    /*  */ 
    DoubleField* field1 = new DoubleField
        (
          Merger::UNDIST_FOCAL_ID,
          offsetof(Merger, mUndistFocal),
          0.9,
          "Undist Focal",
          "Undist Focal",
          "Undist Focal",
          true,
         0,
         99999
        );
    field1->widgetHint=BaseField::SPIN_BOX;
    field1->suffixHint="mm";
    field1->precision=3;
    fields().push_back(field1);
    /*  */ 
    DoubleField* field2 = new DoubleField
        (
          Merger::MM_TO_PIXEL_ID,
          offsetof(Merger, mMMToPixel),
          400,
          "MM to Pixel",
          "MM to Pixel",
          "MM to Pixel",
          true,
         0,
         99999
        );
    field2->widgetHint=BaseField::SPIN_BOX;
    field2->suffixHint="pix";
    field2->precision=3;
    fields().push_back(field2);
    /*  */ 
    IntField* field3 = new IntField
        (
          Merger::DISTORTION_OVERSHOOT_ID,
          offsetof(Merger, mDistortionOvershoot),
          0,
          "Distortion Overshoot",
          "Distortion Overshoot",
          "Distortion Overshoot",
          true,
         0,
         99999
        );
    field3->suffixHint="pix";
    fields().push_back(field3);
    /*  */ 
    BoolField* field4 = new BoolField
        (
          Merger::SHOWMASK_ID,
          offsetof(Merger, mShowMask),
          true,
          "showMask",
          "showMask",
          "showMask"
        );
    field4->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field4);
    /*  */ 
    BoolField* field5 = new BoolField
        (
          Merger::BILINEAR_ID,
          offsetof(Merger, mBilinear),
          false,
          "bilinear",
          "bilinear",
          "bilinear"
        );
    field5->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field5);
    /*  */ 
    BoolField* field6 = new BoolField
        (
          Merger::SEPARATE_VIEW_ID,
          offsetof(Merger, mSeparateView),
          false,
          "Separate View",
          "Separate View",
          "Separate View"
        );
    field6->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field6);
    /*  */ 
    BoolField* field7 = new BoolField
        (
          Merger::DRAW_CAR_ID,
          offsetof(Merger, mDrawCar),
          true,
          "Draw Car",
          "Draw Car",
          "Draw Car"
        );
    field7->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field7);
    /*  */ 
    IntField* field8 = new IntField
        (
          Merger::FRAMETOUNDIST_ID,
          offsetof(Merger, mFrameToUndist),
          0,
          "frameToUndist",
          "frameToUndist",
          "frameToUndist",
          true,
         0,
         3
        );
    fields().push_back(field8);
    /*  */ 
    DoubleField* field9 = new DoubleField
        (
          Merger::OUT_SIZE_H_ID,
          offsetof(Merger, mOutSizeH),
          1000,
          "Out Size H",
          "Out Size H",
          "Out Size H",
          true,
         10,
         99999
        );
    field9->widgetHint=BaseField::SPIN_BOX;
    field9->suffixHint="px";
    field9->precision=2;
    fields().push_back(field9);
    /*  */ 
    DoubleField* field10 = new DoubleField
        (
          Merger::OUT_PHY_SIZE_L_ID,
          offsetof(Merger, mOutPhySizeL),
          200,
          "Out Phy Size L",
          "Out Phy Size L",
          "Out Phy Size L",
          true,
         1,
         99999
        );
    field10->widgetHint=BaseField::SPIN_BOX;
    field10->suffixHint="dm";
    field10->precision=2;
    fields().push_back(field10);
    /*  */ 
    DoubleField* field11 = new DoubleField
        (
          Merger::OUT_PHY_SIZE_W_ID,
          offsetof(Merger, mOutPhySizeW),
          200,
          "Out Phy Size W",
          "Out Phy Size W",
          "Out Phy Size W",
          true,
         1,
         99999
        );
    field11->widgetHint=BaseField::SPIN_BOX;
    field11->suffixHint="dm";
    field11->precision=2;
    fields().push_back(field11);
    /*  */ 
    DoubleField* field12 = new DoubleField
        (
          Merger::GROUND_Z_ID,
          offsetof(Merger, mGroundZ),
          -20,
          "ground Z",
          "ground Z",
          "ground Z",
          true,
         -99999,
         99999
        );
    field12->widgetHint=BaseField::SPIN_BOX;
    field12->precision=2;
    fields().push_back(field12);
    /*  */ 
    DoubleField* field13 = new DoubleField
        (
          Merger::FOV1_ID,
          offsetof(Merger, mFOV1),
          120,
          "FOV1",
          "FOV1",
          "FOV1",
          true,
         1,
         360
        );
    field13->widgetHint=BaseField::SPIN_BOX;
    field13->suffixHint="deg";
    field13->precision=2;
    fields().push_back(field13);
    /*  */ 
    BoolField* field14 = new BoolField
        (
          Merger::SWITCH1_ID,
          offsetof(Merger, mSwitch1),
          true,
          "switch1",
          "switch1",
          "switch1"
        );
    field14->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field14);
    /*  */ 
    CompositeField* field15 = new CompositeField
        (
          Merger::POS1_ID,
          offsetof(Merger, mPos1),
          "pos1",
          "EuclidianMoveParameters",
          "pos1",
          "pos1",
           NULL
        );
    {
        ReflectionDirectory* directory = ReflectionDirectoryHolder::getReflectionDirectory();
        std::string name("Euclidian Move Parameters");
        ReflectionDirectory::iterator it = directory->find(name);
        if(it != directory->end()) {
             field15->reflection = it->second;
        } else {
             printf("Reflection Merger to the subclass Euclidian Move Parameters can't be linked\n");
        }
    }
    fields().push_back(field15);
    /*  */ 
    DoubleField* field16 = new DoubleField
        (
          Merger::FOV2_ID,
          offsetof(Merger, mFOV2),
          120,
          "FOV2",
          "FOV2",
          "FOV2",
          true,
         1,
         360
        );
    field16->widgetHint=BaseField::SPIN_BOX;
    field16->suffixHint="deg";
    field16->precision=2;
    fields().push_back(field16);
    /*  */ 
    BoolField* field17 = new BoolField
        (
          Merger::SWITCH2_ID,
          offsetof(Merger, mSwitch2),
          true,
          "switch2",
          "switch2",
          "switch2"
        );
    field17->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field17);
    /*  */ 
    CompositeField* field18 = new CompositeField
        (
          Merger::POS2_ID,
          offsetof(Merger, mPos2),
          "pos2",
          "EuclidianMoveParameters",
          "pos2",
          "pos2",
           NULL
        );
    {
        ReflectionDirectory* directory = ReflectionDirectoryHolder::getReflectionDirectory();
        std::string name("Euclidian Move Parameters");
        ReflectionDirectory::iterator it = directory->find(name);
        if(it != directory->end()) {
             field18->reflection = it->second;
        } else {
             printf("Reflection Merger to the subclass Euclidian Move Parameters can't be linked\n");
        }
    }
    fields().push_back(field18);
    /*  */ 
    DoubleField* field19 = new DoubleField
        (
          Merger::FOV3_ID,
          offsetof(Merger, mFOV3),
          120,
          "FOV3",
          "FOV3",
          "FOV3",
          true,
         1,
         360
        );
    field19->widgetHint=BaseField::SPIN_BOX;
    field19->suffixHint="deg";
    field19->precision=2;
    fields().push_back(field19);
    /*  */ 
    BoolField* field20 = new BoolField
        (
          Merger::SWITCH3_ID,
          offsetof(Merger, mSwitch3),
          true,
          "switch3",
          "switch3",
          "switch3"
        );
    field20->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field20);
    /*  */ 
    CompositeField* field21 = new CompositeField
        (
          Merger::POS3_ID,
          offsetof(Merger, mPos3),
          "pos3",
          "EuclidianMoveParameters",
          "pos3",
          "pos3",
           NULL
        );
    {
        ReflectionDirectory* directory = ReflectionDirectoryHolder::getReflectionDirectory();
        std::string name("Euclidian Move Parameters");
        ReflectionDirectory::iterator it = directory->find(name);
        if(it != directory->end()) {
             field21->reflection = it->second;
        } else {
             printf("Reflection Merger to the subclass Euclidian Move Parameters can't be linked\n");
        }
    }
    fields().push_back(field21);
    /*  */ 
    DoubleField* field22 = new DoubleField
        (
          Merger::FOV4_ID,
          offsetof(Merger, mFOV4),
          120,
          "FOV4",
          "FOV4",
          "FOV4",
          true,
         1,
         360
        );
    field22->widgetHint=BaseField::SPIN_BOX;
    field22->suffixHint="deg";
    field22->precision=2;
    fields().push_back(field22);
    /*  */ 
    BoolField* field23 = new BoolField
        (
          Merger::SWITCH4_ID,
          offsetof(Merger, mSwitch4),
          true,
          "switch4",
          "switch4",
          "switch4"
        );
    field23->widgetHint=BaseField::CHECK_BOX;
    fields().push_back(field23);
    /*  */ 
    CompositeField* field24 = new CompositeField
        (
          Merger::POS4_ID,
          offsetof(Merger, mPos4),
          "pos4",
          "EuclidianMoveParameters",
          "pos4",
          "pos4",
           NULL
        );
    {
        ReflectionDirectory* directory = ReflectionDirectoryHolder::getReflectionDirectory();
        std::string name("Euclidian Move Parameters");
        ReflectionDirectory::iterator it = directory->find(name);
        if(it != directory->end()) {
             field24->reflection = it->second;
        } else {
             printf("Reflection Merger to the subclass Euclidian Move Parameters can't be linked\n");
        }
    }
    fields().push_back(field24);
    /*  */ 
    ReflectionDirectory &directory = *ReflectionDirectoryHolder::getReflectionDirectory();
    directory[std::string("Merger")]= &reflection;
   return 0;
}
int Merger::relinkCompositeFields()
{
    {
        ReflectionDirectory* directory = ReflectionDirectoryHolder::getReflectionDirectory();
        std::string name("Euclidian Move Parameters");
        ReflectionDirectory::iterator it = directory->find(name);
        if(it != directory->end()) {
             const CompositeField* field = static_cast<const CompositeField*>(getReflection()->fields[15]);
             const_cast<CompositeField*>(field)->reflection = it->second;
        } else {
             printf("Reflection Merger to the subclass Euclidian Move Parameters can't be linked\n");
        }
    }
    {
        ReflectionDirectory* directory = ReflectionDirectoryHolder::getReflectionDirectory();
        std::string name("Euclidian Move Parameters");
        ReflectionDirectory::iterator it = directory->find(name);
        if(it != directory->end()) {
             const CompositeField* field = static_cast<const CompositeField*>(getReflection()->fields[18]);
             const_cast<CompositeField*>(field)->reflection = it->second;
        } else {
             printf("Reflection Merger to the subclass Euclidian Move Parameters can't be linked\n");
        }
    }
    {
        ReflectionDirectory* directory = ReflectionDirectoryHolder::getReflectionDirectory();
        std::string name("Euclidian Move Parameters");
        ReflectionDirectory::iterator it = directory->find(name);
        if(it != directory->end()) {
             const CompositeField* field = static_cast<const CompositeField*>(getReflection()->fields[21]);
             const_cast<CompositeField*>(field)->reflection = it->second;
        } else {
             printf("Reflection Merger to the subclass Euclidian Move Parameters can't be linked\n");
        }
    }
    {
        ReflectionDirectory* directory = ReflectionDirectoryHolder::getReflectionDirectory();
        std::string name("Euclidian Move Parameters");
        ReflectionDirectory::iterator it = directory->find(name);
        if(it != directory->end()) {
             const CompositeField* field = static_cast<const CompositeField*>(getReflection()->fields[24]);
             const_cast<CompositeField*>(field)->reflection = it->second;
        } else {
             printf("Reflection Merger to the subclass Euclidian Move Parameters can't be linked\n");
        }
    }
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


