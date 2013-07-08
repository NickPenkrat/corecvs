#ifndef MASKING_PARAMETERS_H_
#define MASKING_PARAMETERS_H_
/**
 * \file maskingParameters.h
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

using namespace core3vi;

/*
 *  Additional includes for Pointer Types.
 */

namespace core3vi {
}
/*
 *  Additional includes for enum section.
 */

/**
 * \brief Masking Parameters 
 * Masking Parameters 
 **/
class MaskingParameters : public BaseReflection<MaskingParameters>
{
public:
    enum FieldId {
        INVERT_ID,
        MASKING_PARAMETERS_FIELD_ID_NUM
    };

    /** Section with variables */

    /** 
     * \brief Invert 
     * Invert 
     */
    bool mInvert;

    /** Static fields init function, this is used for "dynamic" field initialization */ 
    static int staticInit();

    /** Section with getters */
    const void *getPtrById(int fieldId) const
    {
        return (const unsigned char *)(this) + fields()[fieldId]->offset;
    }
    bool invert() const
    {
        return mInvert;
    }

    /* Section with setters */
    void setInvert(bool invert)
    {
        mInvert = invert;
    }

    /* Section with embedded classes */
    /* visitor pattern - http://en.wikipedia.org/wiki/Visitor_pattern */
template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(mInvert,                    static_cast<const BoolField *>    (fields()[INVERT_ID]));
    }

    MaskingParameters()
    {
        DefaultSetter setter;
        accept(setter);
    }

    MaskingParameters(
          bool invert
    )
    {
        mInvert = invert;
    }

    friend ostream& operator << (ostream &out, MaskingParameters &toSave)
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
#endif  //MASKING_PARAMETERS_H_
