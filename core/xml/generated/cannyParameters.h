#ifndef CANNY_PARAMETERS_H_
#define CANNY_PARAMETERS_H_
/**
 * \file cannyParameters.h
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

/**
 * \brief Canny Parameters 
 * Canny Parameters 
 **/
class CannyParameters : public corecvs::BaseReflection<CannyParameters>
{
public:
    enum FieldId {
        SHOULD_EDGE_DETECT_ID,
        MINIMUM_THRESHOLD_ID,
        MAXIMUM_THRESHOLD_ID,
        CANNY_PARAMETERS_FIELD_ID_NUM
    };

    /** Section with variables */

    /** 
     * \brief Should edge detect 
     * Should edge detect 
     */
    bool mShouldEdgeDetect;

    /** 
     * \brief Minimum threshold 
     * Minimum threshold 
     */
    int mMinimumThreshold;

    /** 
     * \brief Maximum threshold 
     * Maximum threshold 
     */
    int mMaximumThreshold;

    /** Static fields init function, this is used for "dynamic" field initialization */ 
    static int staticInit();

    static int relinkCompositeFields();

    /** Section with getters */
    const void *getPtrById(int fieldId) const
    {
        return (const unsigned char *)(this) + fields()[fieldId]->offset;
    }
    bool shouldEdgeDetect() const
    {
        return mShouldEdgeDetect;
    }

    int minimumThreshold() const
    {
        return mMinimumThreshold;
    }

    int maximumThreshold() const
    {
        return mMaximumThreshold;
    }

    /* Section with setters */
    void setShouldEdgeDetect(bool shouldEdgeDetect)
    {
        mShouldEdgeDetect = shouldEdgeDetect;
    }

    void setMinimumThreshold(int minimumThreshold)
    {
        mMinimumThreshold = minimumThreshold;
    }

    void setMaximumThreshold(int maximumThreshold)
    {
        mMaximumThreshold = maximumThreshold;
    }

    /* Section with embedded classes */
    /* visitor pattern - http://en.wikipedia.org/wiki/Visitor_pattern */
template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(mShouldEdgeDetect,          static_cast<const corecvs::BoolField *>(fields()[SHOULD_EDGE_DETECT_ID]));
        visitor.visit(mMinimumThreshold,          static_cast<const corecvs::IntField *>(fields()[MINIMUM_THRESHOLD_ID]));
        visitor.visit(mMaximumThreshold,          static_cast<const corecvs::IntField *>(fields()[MAXIMUM_THRESHOLD_ID]));
    }

    CannyParameters()
    {
        corecvs::DefaultSetter setter;
        accept(setter);
    }

    CannyParameters(
          bool shouldEdgeDetect
        , int minimumThreshold
        , int maximumThreshold
    )
    {
        mShouldEdgeDetect = shouldEdgeDetect;
        mMinimumThreshold = minimumThreshold;
        mMaximumThreshold = maximumThreshold;
    }

    bool operator ==(const CannyParameters &other) const 
    {
        if ( !(this->mShouldEdgeDetect == other.mShouldEdgeDetect)) return false;
        if ( !(this->mMinimumThreshold == other.mMinimumThreshold)) return false;
        if ( !(this->mMaximumThreshold == other.mMaximumThreshold)) return false;
        return true;
    }
    friend std::ostream& operator << (std::ostream &out, CannyParameters &toSave)
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
#endif  //CANNY_PARAMETERS_H_
