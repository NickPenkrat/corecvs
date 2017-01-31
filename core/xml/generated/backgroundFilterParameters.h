#ifndef BACKGROUND_FILTER_PARAMETERS_H_
#define BACKGROUND_FILTER_PARAMETERS_H_
/**
 * \file backgroundFilterParameters.h
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
 * \brief Background Filter Parameters 
 * Background Filter Parameters 
 **/
class BackgroundFilterParameters : public corecvs::BaseReflection<BackgroundFilterParameters>
{
public:
    enum FieldId {
        THRESHOLD_ID,
        BACKGROUND_FILTER_PARAMETERS_FIELD_ID_NUM
    };

    /** Section with variables */

    /** 
     * \brief Threshold 
     * Threshold 
     */
    int mThreshold;

    /** Static fields init function, this is used for "dynamic" field initialization */ 
    static int staticInit();

    /** Section with getters */
    const void *getPtrById(int fieldId) const
    {
        return (const unsigned char *)(this) + fields()[fieldId]->offset;
    }
    int threshold() const
    {
        return mThreshold;
    }

    /* Section with setters */
    void setThreshold(int threshold)
    {
        mThreshold = threshold;
    }

    /* Section with embedded classes */
    /* visitor pattern - http://en.wikipedia.org/wiki/Visitor_pattern */
template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(mThreshold,                 static_cast<const corecvs::IntField *>(fields()[THRESHOLD_ID]));
    }

    BackgroundFilterParameters()
    {
        corecvs::DefaultSetter setter;
        accept(setter);
    }

    BackgroundFilterParameters(
          int threshold
    )
    {
        mThreshold = threshold;
    }

    friend std::ostream& operator << (std::ostream &out, BackgroundFilterParameters &toSave)
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
#endif  //BACKGROUND_FILTER_PARAMETERS_H_
