#ifndef RANSAC_PARAMETERS_H_
#define RANSAC_PARAMETERS_H_
/**
 * \file ransacParameters.h
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include "reflection.h"
#include "defaultSetter.h"
#include "printerVisitor.h"

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
 * \brief Ransac Parameters 
 * Ransac Parameters 
 **/
class RansacParameters : public corecvs::BaseReflection<RansacParameters>
{
public:
    enum FieldId {
        ITERATIONS_NUMBER_ID,
        USE_MEDIAN_ID,
        INLIERS_PERCENT_ID,
        INLIER_THRESHOLD_ID,
        RANSAC_PARAMETERS_FIELD_ID_NUM
    };

    /** Section with variables */

    /** 
     * \brief Iterations Number 
     * Iterations Number 
     */
    int mIterationsNumber;

    /** 
     * \brief Use Median 
     * Use Median 
     */
    bool mUseMedian;

    /** 
     * \brief Inliers Percent 
     * Inliers Percent 
     */
    double mInliersPercent;

    /** 
     * \brief Inlier Threshold 
     * Inlier Threshold 
     */
    double mInlierThreshold;

    /** Static fields init function, this is used for "dynamic" field initialization */ 
    static int staticInit();

    static int relinkCompositeFields();

    /** Section with getters */
    const void *getPtrById(int fieldId) const
    {
        return (const unsigned char *)(this) + fields()[fieldId]->offset;
    }
    int iterationsNumber() const
    {
        return mIterationsNumber;
    }

    bool useMedian() const
    {
        return mUseMedian;
    }

    double inliersPercent() const
    {
        return mInliersPercent;
    }

    double inlierThreshold() const
    {
        return mInlierThreshold;
    }

    /* Section with setters */
    void setIterationsNumber(int iterationsNumber)
    {
        mIterationsNumber = iterationsNumber;
    }

    void setUseMedian(bool useMedian)
    {
        mUseMedian = useMedian;
    }

    void setInliersPercent(double inliersPercent)
    {
        mInliersPercent = inliersPercent;
    }

    void setInlierThreshold(double inlierThreshold)
    {
        mInlierThreshold = inlierThreshold;
    }

    /* Section with embedded classes */
    /* visitor pattern - http://en.wikipedia.org/wiki/Visitor_pattern */
template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(mIterationsNumber,          static_cast<const corecvs::IntField *>(fields()[ITERATIONS_NUMBER_ID]));
        visitor.visit(mUseMedian,                 static_cast<const corecvs::BoolField *>(fields()[USE_MEDIAN_ID]));
        visitor.visit(mInliersPercent,            static_cast<const corecvs::DoubleField *>(fields()[INLIERS_PERCENT_ID]));
        visitor.visit(mInlierThreshold,           static_cast<const corecvs::DoubleField *>(fields()[INLIER_THRESHOLD_ID]));
    }

    RansacParameters()
    {
        corecvs::DefaultSetter setter;
        accept(setter);
    }

    RansacParameters(
          int iterationsNumber
        , bool useMedian
        , double inliersPercent
        , double inlierThreshold
    )
    {
        mIterationsNumber = iterationsNumber;
        mUseMedian = useMedian;
        mInliersPercent = inliersPercent;
        mInlierThreshold = inlierThreshold;
    }

    friend std::ostream& operator << (std::ostream &out, RansacParameters &toSave)
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
#endif  //RANSAC_PARAMETERS_H_
