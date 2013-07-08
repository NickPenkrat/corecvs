#ifndef DUMMY_PROVIDER_PARAMETERS_H_
#define DUMMY_PROVIDER_PARAMETERS_H_
/**
 * \file dummyProviderParameters.h
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
 * \brief Dummy Provider Parameters 
 * Dummy Provider Parameters 
 **/
class DummyProviderParameters : public BaseReflection<DummyProviderParameters>
{
public:
    enum FieldId {
        PATH_ID,
        UNKNOWN_VALUE_ID,
        PLANE_ZERO_ID,
        PLANE_MAX_ID,
        DUMMY_PROVIDER_PARAMETERS_FIELD_ID_NUM
    };

    /** Section with variables */

    /** 
     * \brief Path 
     * Path 
     */
    std::string mPath;

    /** 
     * \brief Unknown Value 
     * Unknown Value 
     */
    int mUnknownValue;

    /** 
     * \brief Plane Zero 
     * Plane Zero 
     */
    int mPlaneZero;

    /** 
     * \brief Plane Max 
     * Plane Max 
     */
    int mPlaneMax;

    /** Static fields init function, this is used for "dynamic" field initialization */ 
    static int staticInit();

    /** Section with getters */
    const void *getPtrById(int fieldId) const
    {
        return (const unsigned char *)(this) + fields()[fieldId]->offset;
    }
    std::string path() const
    {
        return mPath;
    }

    int unknownValue() const
    {
        return mUnknownValue;
    }

    int planeZero() const
    {
        return mPlaneZero;
    }

    int planeMax() const
    {
        return mPlaneMax;
    }

    /* Section with setters */
    void setPath(std::string path)
    {
        mPath = path;
    }

    void setUnknownValue(int unknownValue)
    {
        mUnknownValue = unknownValue;
    }

    void setPlaneZero(int planeZero)
    {
        mPlaneZero = planeZero;
    }

    void setPlaneMax(int planeMax)
    {
        mPlaneMax = planeMax;
    }

    /* Section with embedded classes */
    /* visitor pattern - http://en.wikipedia.org/wiki/Visitor_pattern */
template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(mPath,                      static_cast<const StringField *>  (fields()[PATH_ID]));
        visitor.visit(mUnknownValue,              static_cast<const IntField *>     (fields()[UNKNOWN_VALUE_ID]));
        visitor.visit(mPlaneZero,                 static_cast<const IntField *>     (fields()[PLANE_ZERO_ID]));
        visitor.visit(mPlaneMax,                  static_cast<const IntField *>     (fields()[PLANE_MAX_ID]));
    }

    DummyProviderParameters()
    {
        DefaultSetter setter;
        accept(setter);
    }

    DummyProviderParameters(
          std::string path
        , int unknownValue
        , int planeZero
        , int planeMax
    )
    {
        mPath = path;
        mUnknownValue = unknownValue;
        mPlaneZero = planeZero;
        mPlaneMax = planeMax;
    }

    friend ostream& operator << (ostream &out, DummyProviderParameters &toSave)
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
#endif  //DUMMY_PROVIDER_PARAMETERS_H_
