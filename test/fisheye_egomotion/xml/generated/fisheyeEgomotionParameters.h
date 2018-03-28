#ifndef FISHEYE_EGOMOTION_PARAMETERS_H_
#define FISHEYE_EGOMOTION_PARAMETERS_H_
/**
 * \file fisheyeEgomotionParameters.h
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 * Generated from fisheye.xml
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

// using namespace corecvs;

/*
 *  Additional includes for Pointer Types.
 */

// namespace corecvs {
// }
/*
 *  Additional includes for enum section.
 */

/**
 * \brief Fisheye Egomotion Parameters 
 * Fisheye Egomotion Parameters 
 **/
class FisheyeEgomotionParameters : public corecvs::BaseReflection<FisheyeEgomotionParameters>
{
public:
    enum FieldId {
        UNWARP_ID,
        JOINT_ID,
        SCENE_ID,
        CAPS_ID,
        SUBPIXEL_ID,
        INTRINSICS_ID,
        PROVIDER_ID,
        FRAMES_ID,
        SKIPF_ID,
        SCALER_ID,
        RTHRESHOLD_ID,
        DTHRESHOLD_ID,
        RSTHRESHOLD_ID,
        RSTHRESPREC_ID,
        DSTHRESHOLD_ID,
        TRACEPARAMS_ID,
        FISHEYE_EGOMOTION_PARAMETERS_FIELD_ID_NUM
    };

    /** Section with variables */

    /** 
     * \brief Produce unwrapped image 
     * Produce unwrapped image 
     */
    bool mUnwarp;

    /** 
     * \brief Make joint optimsation at the end 
     * Make joint optimsation at the end 
     */
    bool mJoint;

    /** 
     * \brief Output info in CVS .json scene format 
     * Output info in CVS .json scene format 
     */
    bool mScene;

    /** 
     * \brief Print capabilities and exit 
     * Print capabilities and exit 
     */
    bool mCaps;

    /** 
     * \brief Force subpixel flow precision. If applicable 
     * Force subpixel flow precision. If applicable 
     */
    bool mSubpixel;

    /** 
     * \brief intrinsics 
     * intrinsics 
     */
    std::string mIntrinsics;

    /** 
     * \brief which flow provider to use. For options see --caps 
     * which flow provider to use. For options see --caps 
     */
    std::string mProvider;

    /** 
     * \brief How many frames to process 
     * How many frames to process 
     */
    int mFrames;

    /** 
     * \brief Skip n frames every frame 
     * Skip n frames every frame 
     */
    int mSkipf;

    /** 
     * \brief Scale image before processing 
     * Scale image before processing 
     */
    double mScaler;

    /** 
     * \brief Threshold for RANSAC during model estimation 
     * Threshold for RANSAC during model estimation 
     */
    double mRthreshold;

    /** 
     * \brief Threshold for actual detection 
     * Threshold for actual detection 
     */
    double mDthreshold;

    /** 
     * \brief Threshold for Static flow 
     * Threshold for Static flow 
     */
    double mRsthreshold;

    /** 
     * \brief precent to fit Static Thresold  
     * precent to fit Static Thresold  
     */
    double mRsthresprec;

    /** 
     * \brief Threshold for Static actual detection 
     * Threshold for Static actual detection 
     */
    double mDsthreshold;

    /** 
     * \brief Trace parameters of the provider 
     * Trace parameters of the provider 
     */
    bool mTraceParams;

    /** Static fields init function, this is used for "dynamic" field initialization */ 
    static int staticInit();

    static int relinkCompositeFields();

    /** Section with getters */
    const void *getPtrById(int fieldId) const
    {
        return (const unsigned char *)(this) + fields()[fieldId]->offset;
    }
    bool unwarp() const
    {
        return mUnwarp;
    }

    bool joint() const
    {
        return mJoint;
    }

    bool scene() const
    {
        return mScene;
    }

    bool caps() const
    {
        return mCaps;
    }

    bool subpixel() const
    {
        return mSubpixel;
    }

    std::string intrinsics() const
    {
        return mIntrinsics;
    }

    std::string provider() const
    {
        return mProvider;
    }

    int frames() const
    {
        return mFrames;
    }

    int skipf() const
    {
        return mSkipf;
    }

    double scaler() const
    {
        return mScaler;
    }

    double rthreshold() const
    {
        return mRthreshold;
    }

    double dthreshold() const
    {
        return mDthreshold;
    }

    double rsthreshold() const
    {
        return mRsthreshold;
    }

    double rsthresprec() const
    {
        return mRsthresprec;
    }

    double dsthreshold() const
    {
        return mDsthreshold;
    }

    bool traceParams() const
    {
        return mTraceParams;
    }

    /* Section with setters */
    void setUnwarp(bool unwarp)
    {
        mUnwarp = unwarp;
    }

    void setJoint(bool joint)
    {
        mJoint = joint;
    }

    void setScene(bool scene)
    {
        mScene = scene;
    }

    void setCaps(bool caps)
    {
        mCaps = caps;
    }

    void setSubpixel(bool subpixel)
    {
        mSubpixel = subpixel;
    }

    void setIntrinsics(std::string intrinsics)
    {
        mIntrinsics = intrinsics;
    }

    void setProvider(std::string provider)
    {
        mProvider = provider;
    }

    void setFrames(int frames)
    {
        mFrames = frames;
    }

    void setSkipf(int skipf)
    {
        mSkipf = skipf;
    }

    void setScaler(double scaler)
    {
        mScaler = scaler;
    }

    void setRthreshold(double rthreshold)
    {
        mRthreshold = rthreshold;
    }

    void setDthreshold(double dthreshold)
    {
        mDthreshold = dthreshold;
    }

    void setRsthreshold(double rsthreshold)
    {
        mRsthreshold = rsthreshold;
    }

    void setRsthresprec(double rsthresprec)
    {
        mRsthresprec = rsthresprec;
    }

    void setDsthreshold(double dsthreshold)
    {
        mDsthreshold = dsthreshold;
    }

    void setTraceParams(bool traceParams)
    {
        mTraceParams = traceParams;
    }

    /* Section with embedded classes */
    /* visitor pattern - http://en.wikipedia.org/wiki/Visitor_pattern */
template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(mUnwarp,                    static_cast<const corecvs::BoolField *>(fields()[UNWARP_ID]));
        visitor.visit(mJoint,                     static_cast<const corecvs::BoolField *>(fields()[JOINT_ID]));
        visitor.visit(mScene,                     static_cast<const corecvs::BoolField *>(fields()[SCENE_ID]));
        visitor.visit(mCaps,                      static_cast<const corecvs::BoolField *>(fields()[CAPS_ID]));
        visitor.visit(mSubpixel,                  static_cast<const corecvs::BoolField *>(fields()[SUBPIXEL_ID]));
        visitor.visit(mIntrinsics,                static_cast<const corecvs::StringField *>(fields()[INTRINSICS_ID]));
        visitor.visit(mProvider,                  static_cast<const corecvs::StringField *>(fields()[PROVIDER_ID]));
        visitor.visit(mFrames,                    static_cast<const corecvs::IntField *>(fields()[FRAMES_ID]));
        visitor.visit(mSkipf,                     static_cast<const corecvs::IntField *>(fields()[SKIPF_ID]));
        visitor.visit(mScaler,                    static_cast<const corecvs::DoubleField *>(fields()[SCALER_ID]));
        visitor.visit(mRthreshold,                static_cast<const corecvs::DoubleField *>(fields()[RTHRESHOLD_ID]));
        visitor.visit(mDthreshold,                static_cast<const corecvs::DoubleField *>(fields()[DTHRESHOLD_ID]));
        visitor.visit(mRsthreshold,               static_cast<const corecvs::DoubleField *>(fields()[RSTHRESHOLD_ID]));
        visitor.visit(mRsthresprec,               static_cast<const corecvs::DoubleField *>(fields()[RSTHRESPREC_ID]));
        visitor.visit(mDsthreshold,               static_cast<const corecvs::DoubleField *>(fields()[DSTHRESHOLD_ID]));
        visitor.visit(mTraceParams,               static_cast<const corecvs::BoolField *>(fields()[TRACEPARAMS_ID]));
    }

    FisheyeEgomotionParameters()
    {
        corecvs::DefaultSetter setter;
        accept(setter);
    }

    FisheyeEgomotionParameters(
          bool unwarp
        , bool joint
        , bool scene
        , bool caps
        , bool subpixel
        , std::string intrinsics
        , std::string provider
        , int frames
        , int skipf
        , double scaler
        , double rthreshold
        , double dthreshold
        , double rsthreshold
        , double rsthresprec
        , double dsthreshold
        , bool traceParams
    )
    {
        mUnwarp = unwarp;
        mJoint = joint;
        mScene = scene;
        mCaps = caps;
        mSubpixel = subpixel;
        mIntrinsics = intrinsics;
        mProvider = provider;
        mFrames = frames;
        mSkipf = skipf;
        mScaler = scaler;
        mRthreshold = rthreshold;
        mDthreshold = dthreshold;
        mRsthreshold = rsthreshold;
        mRsthresprec = rsthresprec;
        mDsthreshold = dsthreshold;
        mTraceParams = traceParams;
    }

    bool operator ==(const FisheyeEgomotionParameters &other) const 
    {
        if ( !(this->mUnwarp == other.mUnwarp)) return false;
        if ( !(this->mJoint == other.mJoint)) return false;
        if ( !(this->mScene == other.mScene)) return false;
        if ( !(this->mCaps == other.mCaps)) return false;
        if ( !(this->mSubpixel == other.mSubpixel)) return false;
        if ( !(this->mIntrinsics == other.mIntrinsics)) return false;
        if ( !(this->mProvider == other.mProvider)) return false;
        if ( !(this->mFrames == other.mFrames)) return false;
        if ( !(this->mSkipf == other.mSkipf)) return false;
        if ( !(this->mScaler == other.mScaler)) return false;
        if ( !(this->mRthreshold == other.mRthreshold)) return false;
        if ( !(this->mDthreshold == other.mDthreshold)) return false;
        if ( !(this->mRsthreshold == other.mRsthreshold)) return false;
        if ( !(this->mRsthresprec == other.mRsthresprec)) return false;
        if ( !(this->mDsthreshold == other.mDsthreshold)) return false;
        if ( !(this->mTraceParams == other.mTraceParams)) return false;
        return true;
    }
    friend std::ostream& operator << (std::ostream &out, FisheyeEgomotionParameters &toSave)
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
#endif  //FISHEYE_EGOMOTION_PARAMETERS_H_
