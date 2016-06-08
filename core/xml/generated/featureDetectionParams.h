#ifndef FEATURE_DETECTION_PARAMS_H_
#define FEATURE_DETECTION_PARAMS_H_
/**
 * \file featureDetectionParams.h
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
 * \brief Feature Detection Params 
 * Feature Detection Params 
 **/
class FeatureDetectionParams : public BaseReflection<FeatureDetectionParams>
{
public:
    enum FieldId {
        DETECTOR_ID,
        DESCRIPTOR_ID,
        MATCHER_ID,
        B2BTHRESHOLD_ID,
        MATCHF2F_ID,
        PARAMETERS_ID,
        FEATURE_DETECTION_PARAMS_FIELD_ID_NUM
    };

    /** Section with variables */

    /** 
     * \brief detector 
     * detector 
     */
    std::string mDetector;

    /** 
     * \brief descriptor 
     * descriptor 
     */
    std::string mDescriptor;

    /** 
     * \brief matcher 
     * matcher 
     */
    std::string mMatcher;

    /** 
     * \brief b2bThreshold 
     * b2bThreshold 
     */
    double mB2bThreshold;

    /** 
     * \brief matchF2F 
     * matchF2F 
     */
    bool mMatchF2F;

    /** 
     * \brief parameters 
     * Additional parameters 
     */
    std::string mParameters;

    /** Static fields init function, this is used for "dynamic" field initialization */ 
    static int staticInit();

    /** Section with getters */
    const void *getPtrById(int fieldId) const
    {
        return (const unsigned char *)(this) + fields()[fieldId]->offset;
    }
    std::string detector() const
    {
        return mDetector;
    }

    std::string descriptor() const
    {
        return mDescriptor;
    }

    std::string matcher() const
    {
        return mMatcher;
    }

    double b2bThreshold() const
    {
        return mB2bThreshold;
    }

    bool matchF2F() const
    {
        return mMatchF2F;
    }

    std::string parameters() const
    {
        return mParameters;
    }

    /* Section with setters */
    void setDetector(std::string detector)
    {
        mDetector = detector;
    }

    void setDescriptor(std::string descriptor)
    {
        mDescriptor = descriptor;
    }

    void setMatcher(std::string matcher)
    {
        mMatcher = matcher;
    }

    void setB2bThreshold(double b2bThreshold)
    {
        mB2bThreshold = b2bThreshold;
    }

    void setMatchF2F(bool matchF2F)
    {
        mMatchF2F = matchF2F;
    }

    void setParameters(std::string parameters)
    {
        mParameters = parameters;
    }

    /* Section with embedded classes */
    /* visitor pattern - http://en.wikipedia.org/wiki/Visitor_pattern */
template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(mDetector,                  static_cast<const StringField *>  (fields()[DETECTOR_ID]));
        visitor.visit(mDescriptor,                static_cast<const StringField *>  (fields()[DESCRIPTOR_ID]));
        visitor.visit(mMatcher,                   static_cast<const StringField *>  (fields()[MATCHER_ID]));
        visitor.visit(mB2bThreshold,              static_cast<const DoubleField *>  (fields()[B2BTHRESHOLD_ID]));
        visitor.visit(mMatchF2F,                  static_cast<const BoolField *>    (fields()[MATCHF2F_ID]));
        visitor.visit(mParameters,                static_cast<const StringField *>  (fields()[PARAMETERS_ID]));
    }

    FeatureDetectionParams()
    {
        DefaultSetter setter;
        accept(setter);
    }

    FeatureDetectionParams(
          std::string detector
        , std::string descriptor
        , std::string matcher
        , double b2bThreshold
        , bool matchF2F
        , std::string parameters
    )
    {
        mDetector = detector;
        mDescriptor = descriptor;
        mMatcher = matcher;
        mB2bThreshold = b2bThreshold;
        mMatchF2F = matchF2F;
        mParameters = parameters;
    }

    friend ostream& operator << (ostream &out, FeatureDetectionParams &toSave)
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
#endif  //FEATURE_DETECTION_PARAMS_H_
