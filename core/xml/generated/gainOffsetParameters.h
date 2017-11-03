#ifndef GAIN_OFFSET_PARAMETERS_H_
#define GAIN_OFFSET_PARAMETERS_H_
/**
 * \file gainOffsetParameters.h
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
 * \brief Gain Offset Parameters 
 * Gain Offset Parameters 
 **/
class GainOffsetParameters : public corecvs::BaseReflection<GainOffsetParameters>
{
public:
    enum FieldId {
        GAIN_ID,
        OFFSET_ID,
        GAIN_OFFSET_PARAMETERS_FIELD_ID_NUM
    };

    /** Section with variables */

    /** 
     * \brief gain 
     * gain 
     */
    double mGain;

    /** 
     * \brief offset 
     * offset 
     */
    double mOffset;

    /** Static fields init function, this is used for "dynamic" field initialization */ 
    static int staticInit();

    static int relinkCompositeFields();

    /** Section with getters */
    const void *getPtrById(int fieldId) const
    {
        return (const unsigned char *)(this) + fields()[fieldId]->offset;
    }
    double gain() const
    {
        return mGain;
    }

    double offset() const
    {
        return mOffset;
    }

    /* Section with setters */
    void setGain(double gain)
    {
        mGain = gain;
    }

    void setOffset(double offset)
    {
        mOffset = offset;
    }

    /* Section with embedded classes */
    /* visitor pattern - http://en.wikipedia.org/wiki/Visitor_pattern */
template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(mGain,                      static_cast<const corecvs::DoubleField *>(fields()[GAIN_ID]));
        visitor.visit(mOffset,                    static_cast<const corecvs::DoubleField *>(fields()[OFFSET_ID]));
    }

    GainOffsetParameters()
    {
        corecvs::DefaultSetter setter;
        accept(setter);
    }

    GainOffsetParameters(
          double gain
        , double offset
    )
    {
        mGain = gain;
        mOffset = offset;
    }

    friend std::ostream& operator << (std::ostream &out, GainOffsetParameters &toSave)
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
#endif  //GAIN_OFFSET_PARAMETERS_H_
