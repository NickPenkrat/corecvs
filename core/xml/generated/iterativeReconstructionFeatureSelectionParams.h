#ifndef ITERATIVE_RECONSTRUCTION_FEATURE_SELECTION_PARAMS_H_
#define ITERATIVE_RECONSTRUCTION_FEATURE_SELECTION_PARAMS_H_
/**
 * \file iterativeReconstructionFeatureSelectionParams.h
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
#include "featureDetectionParams.h"

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
 * \brief Iterative Reconstruction Feature Selection Params 
 * Iterative Reconstruction Feature Selection Params 
 **/
class IterativeReconstructionFeatureSelectionParams : public BaseReflection<IterativeReconstructionFeatureSelectionParams>
{
public:
    enum FieldId {
        INLIERTHRESHOLD_ID,
        TRACKINLIERTHRESHOLD_ID,
        DISTANCELIMIT_ID,
        FEATUREDETECTIONPARAMS_ID,
        RMSEPRUNINGSCALER_ID,
        MAXPRUNINGSCALER_ID,
        POSTOPTIMIZESCALER_ID,
        SKIPFEATUREDETECTION_ID,
        ITERATIVE_RECONSTRUCTION_FEATURE_SELECTION_PARAMS_FIELD_ID_NUM
    };

    /** Section with variables */

    /** 
     * \brief inlierThreshold 
     * Inlier threshold 
     */
    double mInlierThreshold;

    /** 
     * \brief trackInlierThreshold 
     * Track append threshold 
     */
    double mTrackInlierThreshold;

    /** 
     * \brief distanceLimit 
     * Track distance limit 
     */
    double mDistanceLimit;

    /** 
     * \brief featureDetectionParams 
     * featureDetectionParams 
     */
    FeatureDetectionParams mFeatureDetectionParams;

    /** 
     * \brief rmsePruningScaler 
     * RMSE pruning scaler 
     */
    double mRmsePruningScaler;

    /** 
     * \brief maxPruningScaler 
     * Max pruning scaler 
     */
    double mMaxPruningScaler;

    /** 
     * \brief postOptimizeScaler 
     * Post optimize threshold scaler 
     */
    double mPostOptimizeScaler;

    /** 
     * \brief skipFeatureDetection 
     * Skip feature detection 
     */
    bool mSkipFeatureDetection;

    /** Static fields init function, this is used for "dynamic" field initialization */ 
    static int staticInit();

    /** Section with getters */
    const void *getPtrById(int fieldId) const
    {
        return (const unsigned char *)(this) + fields()[fieldId]->offset;
    }
    double inlierThreshold() const
    {
        return mInlierThreshold;
    }

    double trackInlierThreshold() const
    {
        return mTrackInlierThreshold;
    }

    double distanceLimit() const
    {
        return mDistanceLimit;
    }

    FeatureDetectionParams featureDetectionParams() const
    {
        return mFeatureDetectionParams;
    }

    double rmsePruningScaler() const
    {
        return mRmsePruningScaler;
    }

    double maxPruningScaler() const
    {
        return mMaxPruningScaler;
    }

    double postOptimizeScaler() const
    {
        return mPostOptimizeScaler;
    }

    bool skipFeatureDetection() const
    {
        return mSkipFeatureDetection;
    }

    /* Section with setters */
    void setInlierThreshold(double inlierThreshold)
    {
        mInlierThreshold = inlierThreshold;
    }

    void setTrackInlierThreshold(double trackInlierThreshold)
    {
        mTrackInlierThreshold = trackInlierThreshold;
    }

    void setDistanceLimit(double distanceLimit)
    {
        mDistanceLimit = distanceLimit;
    }

    void setFeatureDetectionParams(FeatureDetectionParams const &featureDetectionParams)
    {
        mFeatureDetectionParams = featureDetectionParams;
    }

    void setRmsePruningScaler(double rmsePruningScaler)
    {
        mRmsePruningScaler = rmsePruningScaler;
    }

    void setMaxPruningScaler(double maxPruningScaler)
    {
        mMaxPruningScaler = maxPruningScaler;
    }

    void setPostOptimizeScaler(double postOptimizeScaler)
    {
        mPostOptimizeScaler = postOptimizeScaler;
    }

    void setSkipFeatureDetection(bool skipFeatureDetection)
    {
        mSkipFeatureDetection = skipFeatureDetection;
    }

    /* Section with embedded classes */
    /* visitor pattern - http://en.wikipedia.org/wiki/Visitor_pattern */
template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(mInlierThreshold,           static_cast<const DoubleField *>  (fields()[INLIERTHRESHOLD_ID]));
        visitor.visit(mTrackInlierThreshold,      static_cast<const DoubleField *>  (fields()[TRACKINLIERTHRESHOLD_ID]));
        visitor.visit(mDistanceLimit,             static_cast<const DoubleField *>  (fields()[DISTANCELIMIT_ID]));
        visitor.visit(mFeatureDetectionParams,    static_cast<const CompositeField *>(fields()[FEATUREDETECTIONPARAMS_ID]));
        visitor.visit(mRmsePruningScaler,         static_cast<const DoubleField *>  (fields()[RMSEPRUNINGSCALER_ID]));
        visitor.visit(mMaxPruningScaler,          static_cast<const DoubleField *>  (fields()[MAXPRUNINGSCALER_ID]));
        visitor.visit(mPostOptimizeScaler,        static_cast<const DoubleField *>  (fields()[POSTOPTIMIZESCALER_ID]));
        visitor.visit(mSkipFeatureDetection,      static_cast<const BoolField *>    (fields()[SKIPFEATUREDETECTION_ID]));
    }

    IterativeReconstructionFeatureSelectionParams()
    {
        DefaultSetter setter;
        accept(setter);
    }

    IterativeReconstructionFeatureSelectionParams(
          double inlierThreshold
        , double trackInlierThreshold
        , double distanceLimit
        , FeatureDetectionParams featureDetectionParams
        , double rmsePruningScaler
        , double maxPruningScaler
        , double postOptimizeScaler
        , bool skipFeatureDetection
    )
    {
        mInlierThreshold = inlierThreshold;
        mTrackInlierThreshold = trackInlierThreshold;
        mDistanceLimit = distanceLimit;
        mFeatureDetectionParams = featureDetectionParams;
        mRmsePruningScaler = rmsePruningScaler;
        mMaxPruningScaler = maxPruningScaler;
        mPostOptimizeScaler = postOptimizeScaler;
        mSkipFeatureDetection = skipFeatureDetection;
    }

    friend ostream& operator << (ostream &out, IterativeReconstructionFeatureSelectionParams &toSave)
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
#endif  //ITERATIVE_RECONSTRUCTION_FEATURE_SELECTION_PARAMS_H_
