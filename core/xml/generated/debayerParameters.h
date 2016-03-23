#ifndef DEBAYER_PARAMETERS_H_
#define DEBAYER_PARAMETERS_H_
/**
 * \file debayerParameters.h
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
#include "debayerMethod.h"

/**
 * \brief Debayer Parameters 
 * Debayer Parameters 
 **/
class DebayerParameters : public BaseReflection<DebayerParameters>
{
public:
    enum FieldId {
        METHOD_ID,
        BAYER_POS_ID,
        DEBAYER_PARAMETERS_FIELD_ID_NUM
    };

    /** Section with variables */

    /** 
     * \brief Method 
     * Method 
     */
    int mMethod;

    /** 
     * \brief Bayer pos 
     * Bayer pos 
     */
    int mBayerPos;

    /** Static fields init function, this is used for "dynamic" field initialization */ 
    static int staticInit();

    /** Section with getters */
    const void *getPtrById(int fieldId) const
    {
        return (const unsigned char *)(this) + fields()[fieldId]->offset;
    }
    DebayerMethod::DebayerMethod method() const
    {
        return static_cast<DebayerMethod::DebayerMethod>(mMethod);
    }

    int bayerPos() const
    {
        return mBayerPos;
    }

    /* Section with setters */
    void setMethod(DebayerMethod::DebayerMethod method)
    {
        mMethod = method;
    }

    void setBayerPos(int bayerPos)
    {
        mBayerPos = bayerPos;
    }

    /* Section with embedded classes */
    /* visitor pattern - http://en.wikipedia.org/wiki/Visitor_pattern */
template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit((int &)mMethod,             static_cast<const EnumField *>    (fields()[METHOD_ID]));
        visitor.visit(mBayerPos,                  static_cast<const IntField *>     (fields()[BAYER_POS_ID]));
    }

    DebayerParameters()
    {
        DefaultSetter setter;
        accept(setter);
    }

    DebayerParameters(
          DebayerMethod::DebayerMethod method
        , int bayerPos
    )
    {
        mMethod = method;
        mBayerPos = bayerPos;
    }

    friend ostream& operator << (ostream &out, DebayerParameters &toSave)
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
#endif  //DEBAYER_PARAMETERS_H_
