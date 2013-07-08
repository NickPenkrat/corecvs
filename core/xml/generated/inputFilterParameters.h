#ifndef INPUT_FILTER_PARAMETERS_H_
#define INPUT_FILTER_PARAMETERS_H_
/**
 * \file inputFilterParameters.h
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
#include "inputType.h"

/**
 * \brief Input Filter Parameters 
 * Input Filter Parameters 
 **/
class InputFilterParameters : public BaseReflection<InputFilterParameters>
{
public:
    enum FieldId {
        INPUT_TYPE_ID,
        INPUT_FILTER_PARAMETERS_FIELD_ID_NUM
    };

    /** Section with variables */

    /** 
     * \brief Input Type 
     * Input Type 
     */
    int mInputType;

    /** Static fields init function, this is used for "dynamic" field initialization */ 
    static int staticInit();

    /** Section with getters */
    const void *getPtrById(int fieldId) const
    {
        return (const unsigned char *)(this) + fields()[fieldId]->offset;
    }
    InputType::InputType inputType() const
    {
        return static_cast<InputType::InputType>(mInputType);
    }

    /* Section with setters */
    void setInputType(InputType::InputType inputType)
    {
        mInputType = inputType;
    }

    /* Section with embedded classes */
    /* visitor pattern - http://en.wikipedia.org/wiki/Visitor_pattern */
template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit((int &)mInputType,          static_cast<const EnumField *>    (fields()[INPUT_TYPE_ID]));
    }

    InputFilterParameters()
    {
        DefaultSetter setter;
        accept(setter);
    }

    InputFilterParameters(
          InputType::InputType inputType
    )
    {
        mInputType = inputType;
    }

    friend ostream& operator << (ostream &out, InputFilterParameters &toSave)
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
#endif  //INPUT_FILTER_PARAMETERS_H_
