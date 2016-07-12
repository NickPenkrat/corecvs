#ifndef PHOTOSTATION_APPEND_PARAMETERS_H_
#define PHOTOSTATION_APPEND_PARAMETERS_H_
/**
 * \file photostationAppendParameters.h
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

using namespace corecvs;

/*
 *  Additional includes for Pointer Types.
 */

namespace corecvs {
}
/*
 *  Additional includes for enum section.
 */

/**
 * \brief Photostation Append Parameters 
 * Photostation Append Parameters 
 **/
class PhotostationAppendParameters : public BaseReflection<PhotostationAppendParameters>
{
public:
    enum FieldId {
        MAXPOSTAPPEND_ID,
        INLIERP3PTHRESHOLD_ID,
        MAXP3PITERATIONS_ID,
        INLIERP6PTHRESHOLD_ID,
        MAXP6PITERATIONS_ID,
        GAMMAP6P_ID,
        SPECULATIVITY_ID,
        MINIMALINLIERCOUNT_ID,
        MAXIMALFAILUREPROBABILITY_ID,
        PHOTOSTATION_APPEND_PARAMETERS_FIELD_ID_NUM
    };

    /** Section with variables */

    /** 
     * \brief maxPostAppend 
     * Maximal post-append iterations 
     */
    int mMaxPostAppend;

    /** 
     * \brief inlierP3PThreshold 
     * Inlier threshold for 3P->pose 
     */
    double mInlierP3PThreshold;

    /** 
     * \brief maxP3PIterations 
     * Maximal ransac iterations for 3P->pose 
     */
    int mMaxP3PIterations;

    /** 
     * \brief inlierP6PThreshold 
     * Inlier threshold for 6P->pose/3P->orientation 
     */
    double mInlierP6PThreshold;

    /** 
     * \brief maxP6PIterations 
     * Maximal ransac iterations for 6P->pose/3P->orientation 
     */
    int mMaxP6PIterations;

    /** 
     * \brief gammaP6P 
     * Target error probability for 6P->pose/3P->orientation 
     */
    double mGammaP6P;

    /** 
     * \brief speculativity 
     * This defines how many multicameras are subject for P3P evaluation at each iteration 
     */
    int mSpeculativity;

    /** 
     * \brief minimalInlierCount 
     * Minimal inlier count for hypotheis acceptance 
     */
    int mMinimalInlierCount;

    /** 
     * \brief maximalFailureProbability 
     * Maximal failure probability for hypothesis acceptance 
     */
    double mMaximalFailureProbability;

    /** Static fields init function, this is used for "dynamic" field initialization */ 
    static int staticInit();

    /** Section with getters */
    const void *getPtrById(int fieldId) const
    {
        return (const unsigned char *)(this) + fields()[fieldId]->offset;
    }
    int maxPostAppend() const
    {
        return mMaxPostAppend;
    }

    double inlierP3PThreshold() const
    {
        return mInlierP3PThreshold;
    }

    int maxP3PIterations() const
    {
        return mMaxP3PIterations;
    }

    double inlierP6PThreshold() const
    {
        return mInlierP6PThreshold;
    }

    int maxP6PIterations() const
    {
        return mMaxP6PIterations;
    }

    double gammaP6P() const
    {
        return mGammaP6P;
    }

    int speculativity() const
    {
        return mSpeculativity;
    }

    int minimalInlierCount() const
    {
        return mMinimalInlierCount;
    }

    double maximalFailureProbability() const
    {
        return mMaximalFailureProbability;
    }

    /* Section with setters */
    void setMaxPostAppend(int maxPostAppend)
    {
        mMaxPostAppend = maxPostAppend;
    }

    void setInlierP3PThreshold(double inlierP3PThreshold)
    {
        mInlierP3PThreshold = inlierP3PThreshold;
    }

    void setMaxP3PIterations(int maxP3PIterations)
    {
        mMaxP3PIterations = maxP3PIterations;
    }

    void setInlierP6PThreshold(double inlierP6PThreshold)
    {
        mInlierP6PThreshold = inlierP6PThreshold;
    }

    void setMaxP6PIterations(int maxP6PIterations)
    {
        mMaxP6PIterations = maxP6PIterations;
    }

    void setGammaP6P(double gammaP6P)
    {
        mGammaP6P = gammaP6P;
    }

    void setSpeculativity(int speculativity)
    {
        mSpeculativity = speculativity;
    }

    void setMinimalInlierCount(int minimalInlierCount)
    {
        mMinimalInlierCount = minimalInlierCount;
    }

    void setMaximalFailureProbability(double maximalFailureProbability)
    {
        mMaximalFailureProbability = maximalFailureProbability;
    }

    /* Section with embedded classes */
    /* visitor pattern - http://en.wikipedia.org/wiki/Visitor_pattern */
template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(mMaxPostAppend,             static_cast<const IntField *>     (fields()[MAXPOSTAPPEND_ID]));
        visitor.visit(mInlierP3PThreshold,        static_cast<const DoubleField *>  (fields()[INLIERP3PTHRESHOLD_ID]));
        visitor.visit(mMaxP3PIterations,          static_cast<const IntField *>     (fields()[MAXP3PITERATIONS_ID]));
        visitor.visit(mInlierP6PThreshold,        static_cast<const DoubleField *>  (fields()[INLIERP6PTHRESHOLD_ID]));
        visitor.visit(mMaxP6PIterations,          static_cast<const IntField *>     (fields()[MAXP6PITERATIONS_ID]));
        visitor.visit(mGammaP6P,                  static_cast<const DoubleField *>  (fields()[GAMMAP6P_ID]));
        visitor.visit(mSpeculativity,             static_cast<const IntField *>     (fields()[SPECULATIVITY_ID]));
        visitor.visit(mMinimalInlierCount,        static_cast<const IntField *>     (fields()[MINIMALINLIERCOUNT_ID]));
        visitor.visit(mMaximalFailureProbability, static_cast<const DoubleField *>  (fields()[MAXIMALFAILUREPROBABILITY_ID]));
    }

    PhotostationAppendParameters()
    {
        DefaultSetter setter;
        accept(setter);
    }

    PhotostationAppendParameters(
          int maxPostAppend
        , double inlierP3PThreshold
        , int maxP3PIterations
        , double inlierP6PThreshold
        , int maxP6PIterations
        , double gammaP6P
        , int speculativity
        , int minimalInlierCount
        , double maximalFailureProbability
    )
    {
        mMaxPostAppend = maxPostAppend;
        mInlierP3PThreshold = inlierP3PThreshold;
        mMaxP3PIterations = maxP3PIterations;
        mInlierP6PThreshold = inlierP6PThreshold;
        mMaxP6PIterations = maxP6PIterations;
        mGammaP6P = gammaP6P;
        mSpeculativity = speculativity;
        mMinimalInlierCount = minimalInlierCount;
        mMaximalFailureProbability = maximalFailureProbability;
    }

    friend ostream& operator << (ostream &out, PhotostationAppendParameters &toSave)
    {
        PrinterVisitor printer(out);
        toSave.accept<PrinterVisitor>(printer);
        return out;
    }

    void print ()
    {
        cout << *this;
    }
};
#endif  //PHOTOSTATION_APPEND_PARAMETERS_H_
