#ifndef PHOTOSTATION_PLACER_PARAMS_BASE_H_
#define PHOTOSTATION_PLACER_PARAMS_BASE_H_
/**
 * \file photostationPlacerParamsBase.h
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
 * \brief Photostation Placer Params Base 
 * Photostation Placer Params Base 
 **/
class PhotostationPlacerParamsBase : public BaseReflection<PhotostationPlacerParamsBase>
{
public:
    enum FieldId {
        FORCEGPS_ID,
        SPECULATIVITY_ID,
        MINIMALINLIERCOUNT_ID,
        MAXIMALFAILUREPROBABILITY_ID,
        PHOTOSTATION_PLACER_PARAMS_BASE_FIELD_ID_NUM
    };

    /** Section with variables */

    /** 
     * \brief forceGps 
     * Enforce gps locations 
     */
    bool mForceGps;

    /** 
     * \brief speculativity 
     * This defines how many multicameras are subject for P3P evaluation at each iteration 
     */
    int mSpeculativity;

    /** 
     * \brief minimalInlierCount 
     * minimalInlierCount 
     */
    int mMinimalInlierCount;

    /** 
     * \brief maximalFailureProbability 
     * maximalFailureProbability 
     */
    double mMaximalFailureProbability;

    /** Static fields init function, this is used for "dynamic" field initialization */ 
    static int staticInit();

    /** Section with getters */
    const void *getPtrById(int fieldId) const
    {
        return (const unsigned char *)(this) + fields()[fieldId]->offset;
    }
    bool forceGps() const
    {
        return mForceGps;
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
    void setForceGps(bool forceGps)
    {
        mForceGps = forceGps;
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
        visitor.visit(mForceGps,                  static_cast<const BoolField *>    (fields()[FORCEGPS_ID]));
        visitor.visit(mSpeculativity,             static_cast<const IntField *>     (fields()[SPECULATIVITY_ID]));
        visitor.visit(mMinimalInlierCount,        static_cast<const IntField *>     (fields()[MINIMALINLIERCOUNT_ID]));
        visitor.visit(mMaximalFailureProbability, static_cast<const DoubleField *>  (fields()[MAXIMALFAILUREPROBABILITY_ID]));
    }

    PhotostationPlacerParamsBase()
    {
        DefaultSetter setter;
        accept(setter);
    }

    PhotostationPlacerParamsBase(
          bool forceGps
        , int speculativity
        , int minimalInlierCount
        , double maximalFailureProbability
    )
    {
        mForceGps = forceGps;
        mSpeculativity = speculativity;
        mMinimalInlierCount = minimalInlierCount;
        mMaximalFailureProbability = maximalFailureProbability;
    }

    friend ostream& operator << (ostream &out, PhotostationPlacerParamsBase &toSave)
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
#endif  //PHOTOSTATION_PLACER_PARAMS_BASE_H_
