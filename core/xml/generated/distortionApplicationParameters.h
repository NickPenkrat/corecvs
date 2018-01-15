#ifndef DISTORTION_APPLICATION_PARAMETERS_H_
#define DISTORTION_APPLICATION_PARAMETERS_H_
/**
 * \file distortionApplicationParameters.h
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
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
#include "core/xml/generated/distortionResizePolicy.h"

/**
 * \brief Distortion Application Parameters 
 * Distortion Application Parameters 
 **/
class DistortionApplicationParameters : public corecvs::BaseReflection<DistortionApplicationParameters>
{
public:
    enum FieldId {
        FORCE_SCALE_ID,
        ADOPT_SCALE_ID,
        RESIZE_POLICY_ID,
        NEW_H_ID,
        NEW_W_ID,
        DISTORTION_APPLICATION_PARAMETERS_FIELD_ID_NUM
    };

    /** Section with variables */

    /** 
     * \brief Force Scale 
     * Force Scale 
     */
    bool mForceScale;

    /** 
     * \brief Adopt Scale 
     * Adopt Scale 
     */
    bool mAdoptScale;

    /** 
     * \brief Resize policy 
     * Resize policy 
     */
    int mResizePolicy;

    /** 
     * \brief New H 
     * New H 
     */
    int mNewH;

    /** 
     * \brief New W 
     * New W 
     */
    int mNewW;

    /** Static fields init function, this is used for "dynamic" field initialization */ 
    static int staticInit();

    static int relinkCompositeFields();

    /** Section with getters */
    const void *getPtrById(int fieldId) const
    {
        return (const unsigned char *)(this) + fields()[fieldId]->offset;
    }
    bool forceScale() const
    {
        return mForceScale;
    }

    bool adoptScale() const
    {
        return mAdoptScale;
    }

    DistortionResizePolicy::DistortionResizePolicy resizePolicy() const
    {
        return static_cast<DistortionResizePolicy::DistortionResizePolicy>(mResizePolicy);
    }

    int newH() const
    {
        return mNewH;
    }

    int newW() const
    {
        return mNewW;
    }

    /* Section with setters */
    void setForceScale(bool forceScale)
    {
        mForceScale = forceScale;
    }

    void setAdoptScale(bool adoptScale)
    {
        mAdoptScale = adoptScale;
    }

    void setResizePolicy(DistortionResizePolicy::DistortionResizePolicy resizePolicy)
    {
        mResizePolicy = resizePolicy;
    }

    void setNewH(int newH)
    {
        mNewH = newH;
    }

    void setNewW(int newW)
    {
        mNewW = newW;
    }

    /* Section with embedded classes */
    /* visitor pattern - http://en.wikipedia.org/wiki/Visitor_pattern */
template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(mForceScale,                static_cast<const corecvs::BoolField *>(fields()[FORCE_SCALE_ID]));
        visitor.visit(mAdoptScale,                static_cast<const corecvs::BoolField *>(fields()[ADOPT_SCALE_ID]));
        visitor.visit((int &)mResizePolicy,       static_cast<const corecvs::EnumField *>(fields()[RESIZE_POLICY_ID]));
        visitor.visit(mNewH,                      static_cast<const corecvs::IntField *>(fields()[NEW_H_ID]));
        visitor.visit(mNewW,                      static_cast<const corecvs::IntField *>(fields()[NEW_W_ID]));
    }

    DistortionApplicationParameters()
    {
        corecvs::DefaultSetter setter;
        accept(setter);
    }

    DistortionApplicationParameters(
          bool forceScale
        , bool adoptScale
        , DistortionResizePolicy::DistortionResizePolicy resizePolicy
        , int newH
        , int newW
    )
    {
        mForceScale = forceScale;
        mAdoptScale = adoptScale;
        mResizePolicy = resizePolicy;
        mNewH = newH;
        mNewW = newW;
    }

    bool operator ==(const DistortionApplicationParameters &other) const 
    {
        if ( !(this->mForceScale == other.mForceScale)) return false;
        if ( !(this->mAdoptScale == other.mAdoptScale)) return false;
        if ( !(this->mResizePolicy == other.mResizePolicy)) return false;
        if ( !(this->mNewH == other.mNewH)) return false;
        if ( !(this->mNewW == other.mNewW)) return false;
        return true;
    }
    friend std::ostream& operator << (std::ostream &out, DistortionApplicationParameters &toSave)
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
#endif  //DISTORTION_APPLICATION_PARAMETERS_H_
