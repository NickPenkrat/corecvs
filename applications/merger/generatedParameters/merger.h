#ifndef MERGER_H_
#define MERGER_H_
/**
 * \file merger.h
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 * Generated from merger.xml
 */

#include "core/reflection/reflection.h"
#include "core/reflection/defaultSetter.h"
#include "core/reflection/printerVisitor.h"

/*
 *  Embed includes.
 */
/*
 *  Additional includes for Composite Types.
 */
#include "core/xml/generated/euclidianMoveParameters.h"

// using namespace corecvs;

/*
 *  Additional includes for Pointer Types.
 */

// namespace corecvs {
// }
/*
 *  Additional includes for enum section.
 */
#include "core/xml/generated/mergerUndistMethod.h"

/**
 * \brief Merger parameters 
 * Merger parameters 
 **/
class Merger : public corecvs::BaseReflection<Merger>
{
public:
    enum FieldId {
        UNDISTMETHOD_ID,
        UNDIST_FOCAL_ID,
        SENSOR_WIDTH_ID,
        DISTORTION_OVERSHOOT_ID,
        SHOWMASK_ID,
        BILINEAR_ID,
        SEPARATE_VIEW_ID,
        DRAW_CAR_ID,
        FRAMETOUNDIST_ID,
        OUT_SIZE_H_ID,
        OUT_PHY_SIZE_L_ID,
        OUT_PHY_SIZE_W_ID,
        GROUND_Z_ID,
        FOV1_ID,
        SWITCH1_ID,
        POS1_ID,
        FOV2_ID,
        SWITCH2_ID,
        POS2_ID,
        FOV3_ID,
        SWITCH3_ID,
        POS3_ID,
        FOV4_ID,
        SWITCH4_ID,
        POS4_ID,
        MERGER_FIELD_ID_NUM
    };

    /** Section with variables */

    /** 
     * \brief undistMethod 
     * undistMethod 
     */
    int mUndistMethod;

    /** 
     * \brief Undist Focal 
     * Undist Focal 
     */
    double mUndistFocal;

    /** 
     * \brief Sensor Width 
     * Sensor Width 
     */
    double mSensorWidth;

    /** 
     * \brief Distortion Overshoot 
     * Distortion Overshoot 
     */
    int mDistortionOvershoot;

    /** 
     * \brief showMask 
     * showMask 
     */
    bool mShowMask;

    /** 
     * \brief bilinear 
     * bilinear 
     */
    bool mBilinear;

    /** 
     * \brief Separate View 
     * Separate View 
     */
    bool mSeparateView;

    /** 
     * \brief Draw Car 
     * Draw Car 
     */
    bool mDrawCar;

    /** 
     * \brief frameToUndist 
     * frameToUndist 
     */
    int mFrameToUndist;

    /** 
     * \brief Out Size H 
     * Out Size H 
     */
    double mOutSizeH;

    /** 
     * \brief Out Phy Size L 
     * Out Phy Size L 
     */
    double mOutPhySizeL;

    /** 
     * \brief Out Phy Size W 
     * Out Phy Size W 
     */
    double mOutPhySizeW;

    /** 
     * \brief ground Z 
     * ground Z 
     */
    double mGroundZ;

    /** 
     * \brief FOV1 
     * FOV1 
     */
    double mFOV1;

    /** 
     * \brief switch1 
     * switch1 
     */
    bool mSwitch1;

    /** 
     * \brief pos1 
     * pos1 
     */
    EuclidianMoveParameters mPos1;

    /** 
     * \brief FOV2 
     * FOV2 
     */
    double mFOV2;

    /** 
     * \brief switch2 
     * switch2 
     */
    bool mSwitch2;

    /** 
     * \brief pos2 
     * pos2 
     */
    EuclidianMoveParameters mPos2;

    /** 
     * \brief FOV3 
     * FOV3 
     */
    double mFOV3;

    /** 
     * \brief switch3 
     * switch3 
     */
    bool mSwitch3;

    /** 
     * \brief pos3 
     * pos3 
     */
    EuclidianMoveParameters mPos3;

    /** 
     * \brief FOV4 
     * FOV4 
     */
    double mFOV4;

    /** 
     * \brief switch4 
     * switch4 
     */
    bool mSwitch4;

    /** 
     * \brief pos4 
     * pos4 
     */
    EuclidianMoveParameters mPos4;

    /** Static fields init function, this is used for "dynamic" field initialization */ 
    static int staticInit();

    static int relinkCompositeFields();

    /** Section with getters */
    const void *getPtrById(int fieldId) const
    {
        return (const unsigned char *)(this) + fields()[fieldId]->offset;
    }
    MergerUndistMethod::MergerUndistMethod undistMethod() const
    {
        return static_cast<MergerUndistMethod::MergerUndistMethod>(mUndistMethod);
    }

    double undistFocal() const
    {
        return mUndistFocal;
    }

    double sensorWidth() const
    {
        return mSensorWidth;
    }

    int distortionOvershoot() const
    {
        return mDistortionOvershoot;
    }

    bool showMask() const
    {
        return mShowMask;
    }

    bool bilinear() const
    {
        return mBilinear;
    }

    bool separateView() const
    {
        return mSeparateView;
    }

    bool drawCar() const
    {
        return mDrawCar;
    }

    int frameToUndist() const
    {
        return mFrameToUndist;
    }

    double outSizeH() const
    {
        return mOutSizeH;
    }

    double outPhySizeL() const
    {
        return mOutPhySizeL;
    }

    double outPhySizeW() const
    {
        return mOutPhySizeW;
    }

    double groundZ() const
    {
        return mGroundZ;
    }

    double fOV1() const
    {
        return mFOV1;
    }

    bool switch1() const
    {
        return mSwitch1;
    }

    EuclidianMoveParameters pos1() const
    {
        return mPos1;
    }

    double fOV2() const
    {
        return mFOV2;
    }

    bool switch2() const
    {
        return mSwitch2;
    }

    EuclidianMoveParameters pos2() const
    {
        return mPos2;
    }

    double fOV3() const
    {
        return mFOV3;
    }

    bool switch3() const
    {
        return mSwitch3;
    }

    EuclidianMoveParameters pos3() const
    {
        return mPos3;
    }

    double fOV4() const
    {
        return mFOV4;
    }

    bool switch4() const
    {
        return mSwitch4;
    }

    EuclidianMoveParameters pos4() const
    {
        return mPos4;
    }

    /* Section with setters */
    void setUndistMethod(MergerUndistMethod::MergerUndistMethod undistMethod)
    {
        mUndistMethod = undistMethod;
    }

    void setUndistFocal(double undistFocal)
    {
        mUndistFocal = undistFocal;
    }

    void setSensorWidth(double sensorWidth)
    {
        mSensorWidth = sensorWidth;
    }

    void setDistortionOvershoot(int distortionOvershoot)
    {
        mDistortionOvershoot = distortionOvershoot;
    }

    void setShowMask(bool showMask)
    {
        mShowMask = showMask;
    }

    void setBilinear(bool bilinear)
    {
        mBilinear = bilinear;
    }

    void setSeparateView(bool separateView)
    {
        mSeparateView = separateView;
    }

    void setDrawCar(bool drawCar)
    {
        mDrawCar = drawCar;
    }

    void setFrameToUndist(int frameToUndist)
    {
        mFrameToUndist = frameToUndist;
    }

    void setOutSizeH(double outSizeH)
    {
        mOutSizeH = outSizeH;
    }

    void setOutPhySizeL(double outPhySizeL)
    {
        mOutPhySizeL = outPhySizeL;
    }

    void setOutPhySizeW(double outPhySizeW)
    {
        mOutPhySizeW = outPhySizeW;
    }

    void setGroundZ(double groundZ)
    {
        mGroundZ = groundZ;
    }

    void setFOV1(double fOV1)
    {
        mFOV1 = fOV1;
    }

    void setSwitch1(bool switch1)
    {
        mSwitch1 = switch1;
    }

    void setPos1(EuclidianMoveParameters const &pos1)
    {
        mPos1 = pos1;
    }

    void setFOV2(double fOV2)
    {
        mFOV2 = fOV2;
    }

    void setSwitch2(bool switch2)
    {
        mSwitch2 = switch2;
    }

    void setPos2(EuclidianMoveParameters const &pos2)
    {
        mPos2 = pos2;
    }

    void setFOV3(double fOV3)
    {
        mFOV3 = fOV3;
    }

    void setSwitch3(bool switch3)
    {
        mSwitch3 = switch3;
    }

    void setPos3(EuclidianMoveParameters const &pos3)
    {
        mPos3 = pos3;
    }

    void setFOV4(double fOV4)
    {
        mFOV4 = fOV4;
    }

    void setSwitch4(bool switch4)
    {
        mSwitch4 = switch4;
    }

    void setPos4(EuclidianMoveParameters const &pos4)
    {
        mPos4 = pos4;
    }

    /* Section with embedded classes */
    /* visitor pattern - http://en.wikipedia.org/wiki/Visitor_pattern */
template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit((int &)mUndistMethod,       static_cast<const corecvs::EnumField *>(fields()[UNDISTMETHOD_ID]));
        visitor.visit(mUndistFocal,               static_cast<const corecvs::DoubleField *>(fields()[UNDIST_FOCAL_ID]));
        visitor.visit(mSensorWidth,               static_cast<const corecvs::DoubleField *>(fields()[SENSOR_WIDTH_ID]));
        visitor.visit(mDistortionOvershoot,       static_cast<const corecvs::IntField *>(fields()[DISTORTION_OVERSHOOT_ID]));
        visitor.visit(mShowMask,                  static_cast<const corecvs::BoolField *>(fields()[SHOWMASK_ID]));
        visitor.visit(mBilinear,                  static_cast<const corecvs::BoolField *>(fields()[BILINEAR_ID]));
        visitor.visit(mSeparateView,              static_cast<const corecvs::BoolField *>(fields()[SEPARATE_VIEW_ID]));
        visitor.visit(mDrawCar,                   static_cast<const corecvs::BoolField *>(fields()[DRAW_CAR_ID]));
        visitor.visit(mFrameToUndist,             static_cast<const corecvs::IntField *>(fields()[FRAMETOUNDIST_ID]));
        visitor.visit(mOutSizeH,                  static_cast<const corecvs::DoubleField *>(fields()[OUT_SIZE_H_ID]));
        visitor.visit(mOutPhySizeL,               static_cast<const corecvs::DoubleField *>(fields()[OUT_PHY_SIZE_L_ID]));
        visitor.visit(mOutPhySizeW,               static_cast<const corecvs::DoubleField *>(fields()[OUT_PHY_SIZE_W_ID]));
        visitor.visit(mGroundZ,                   static_cast<const corecvs::DoubleField *>(fields()[GROUND_Z_ID]));
        visitor.visit(mFOV1,                      static_cast<const corecvs::DoubleField *>(fields()[FOV1_ID]));
        visitor.visit(mSwitch1,                   static_cast<const corecvs::BoolField *>(fields()[SWITCH1_ID]));
        visitor.visit(mPos1,                      static_cast<const corecvs::CompositeField *>(fields()[POS1_ID]));
        visitor.visit(mFOV2,                      static_cast<const corecvs::DoubleField *>(fields()[FOV2_ID]));
        visitor.visit(mSwitch2,                   static_cast<const corecvs::BoolField *>(fields()[SWITCH2_ID]));
        visitor.visit(mPos2,                      static_cast<const corecvs::CompositeField *>(fields()[POS2_ID]));
        visitor.visit(mFOV3,                      static_cast<const corecvs::DoubleField *>(fields()[FOV3_ID]));
        visitor.visit(mSwitch3,                   static_cast<const corecvs::BoolField *>(fields()[SWITCH3_ID]));
        visitor.visit(mPos3,                      static_cast<const corecvs::CompositeField *>(fields()[POS3_ID]));
        visitor.visit(mFOV4,                      static_cast<const corecvs::DoubleField *>(fields()[FOV4_ID]));
        visitor.visit(mSwitch4,                   static_cast<const corecvs::BoolField *>(fields()[SWITCH4_ID]));
        visitor.visit(mPos4,                      static_cast<const corecvs::CompositeField *>(fields()[POS4_ID]));
    }

    Merger()
    {
        corecvs::DefaultSetter setter;
        accept(setter);
    }

    Merger(
          MergerUndistMethod::MergerUndistMethod undistMethod
        , double undistFocal
        , double sensorWidth
        , int distortionOvershoot
        , bool showMask
        , bool bilinear
        , bool separateView
        , bool drawCar
        , int frameToUndist
        , double outSizeH
        , double outPhySizeL
        , double outPhySizeW
        , double groundZ
        , double fOV1
        , bool switch1
        , EuclidianMoveParameters pos1
        , double fOV2
        , bool switch2
        , EuclidianMoveParameters pos2
        , double fOV3
        , bool switch3
        , EuclidianMoveParameters pos3
        , double fOV4
        , bool switch4
        , EuclidianMoveParameters pos4
    )
    {
        mUndistMethod = undistMethod;
        mUndistFocal = undistFocal;
        mSensorWidth = sensorWidth;
        mDistortionOvershoot = distortionOvershoot;
        mShowMask = showMask;
        mBilinear = bilinear;
        mSeparateView = separateView;
        mDrawCar = drawCar;
        mFrameToUndist = frameToUndist;
        mOutSizeH = outSizeH;
        mOutPhySizeL = outPhySizeL;
        mOutPhySizeW = outPhySizeW;
        mGroundZ = groundZ;
        mFOV1 = fOV1;
        mSwitch1 = switch1;
        mPos1 = pos1;
        mFOV2 = fOV2;
        mSwitch2 = switch2;
        mPos2 = pos2;
        mFOV3 = fOV3;
        mSwitch3 = switch3;
        mPos3 = pos3;
        mFOV4 = fOV4;
        mSwitch4 = switch4;
        mPos4 = pos4;
    }

    bool operator ==(const Merger &other) const 
    {
        if ( !(this->mUndistMethod == other.mUndistMethod)) return false;
        if ( !(this->mUndistFocal == other.mUndistFocal)) return false;
        if ( !(this->mSensorWidth == other.mSensorWidth)) return false;
        if ( !(this->mDistortionOvershoot == other.mDistortionOvershoot)) return false;
        if ( !(this->mShowMask == other.mShowMask)) return false;
        if ( !(this->mBilinear == other.mBilinear)) return false;
        if ( !(this->mSeparateView == other.mSeparateView)) return false;
        if ( !(this->mDrawCar == other.mDrawCar)) return false;
        if ( !(this->mFrameToUndist == other.mFrameToUndist)) return false;
        if ( !(this->mOutSizeH == other.mOutSizeH)) return false;
        if ( !(this->mOutPhySizeL == other.mOutPhySizeL)) return false;
        if ( !(this->mOutPhySizeW == other.mOutPhySizeW)) return false;
        if ( !(this->mGroundZ == other.mGroundZ)) return false;
        if ( !(this->mFOV1 == other.mFOV1)) return false;
        if ( !(this->mSwitch1 == other.mSwitch1)) return false;
        if ( !(this->mPos1 == other.mPos1)) return false;
        if ( !(this->mFOV2 == other.mFOV2)) return false;
        if ( !(this->mSwitch2 == other.mSwitch2)) return false;
        if ( !(this->mPos2 == other.mPos2)) return false;
        if ( !(this->mFOV3 == other.mFOV3)) return false;
        if ( !(this->mSwitch3 == other.mSwitch3)) return false;
        if ( !(this->mPos3 == other.mPos3)) return false;
        if ( !(this->mFOV4 == other.mFOV4)) return false;
        if ( !(this->mSwitch4 == other.mSwitch4)) return false;
        if ( !(this->mPos4 == other.mPos4)) return false;
        return true;
    }
    friend std::ostream& operator << (std::ostream &out, Merger &toSave)
    {
        corecvs::PrinterVisitor printer(out);
        toSave.accept<corecvs::PrinterVisitor>(printer);
        return out;
    }

    void print ()
    {
        std::cout << *this;
    }
};
#endif  //MERGER_H_
