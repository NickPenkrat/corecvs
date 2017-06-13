#ifndef ADDER_SUBSTRACTOR_PARAMETERS_BASE_H_
#define ADDER_SUBSTRACTOR_PARAMETERS_BASE_H_
/**
 * \file adderSubstractorParametersBase.h
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
 * \brief Adder Substractor Parameters Base 
 * Adder Substractor Parameters Base 
 **/
class AdderSubstractorParametersBase : public corecvs::BaseReflection<AdderSubstractorParametersBase>
{
public:
    enum FieldId {
        INPUT1_ID,
        INPUT2_ID,
        PARAMETER_ID,
        OUTPUT1_ID,
        OUTPUT2_ID,
        ADDER_SUBSTRACTOR_PARAMETERS_BASE_FIELD_ID_NUM
    };

    /** Section with variables */

    /** 
     * \brief input1 
     * input1 
     */
    double mInput1;

    /** 
     * \brief input2 
     * input2 
     */
    double mInput2;

    /** 
     * \brief parameter 
     * parameter 
     */
    bool mParameter;

    /** 
     * \brief output1 
     * output1 
     */
    double mOutput1;

    /** 
     * \brief output2 
     * output2 
     */
    double mOutput2;

    /** Static fields init function, this is used for "dynamic" field initialization */ 
    static int staticInit();

    static int relinkCompositeFields();

    /** Section with getters */
    const void *getPtrById(int fieldId) const
    {
        return (const unsigned char *)(this) + fields()[fieldId]->offset;
    }
    double input1() const
    {
        return mInput1;
    }

    double input2() const
    {
        return mInput2;
    }

    bool parameter() const
    {
        return mParameter;
    }

    double output1() const
    {
        return mOutput1;
    }

    double output2() const
    {
        return mOutput2;
    }

    /* Section with setters */
    void setInput1(double input1)
    {
        mInput1 = input1;
    }

    void setInput2(double input2)
    {
        mInput2 = input2;
    }

    void setParameter(bool parameter)
    {
        mParameter = parameter;
    }

    void setOutput1(double output1)
    {
        mOutput1 = output1;
    }

    void setOutput2(double output2)
    {
        mOutput2 = output2;
    }

    /* Section with embedded classes */
    /* visitor pattern - http://en.wikipedia.org/wiki/Visitor_pattern */
template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(mInput1,                    static_cast<const corecvs::DoubleField *>(fields()[INPUT1_ID]));
        visitor.visit(mInput2,                    static_cast<const corecvs::DoubleField *>(fields()[INPUT2_ID]));
        visitor.visit(mParameter,                 static_cast<const corecvs::BoolField *>(fields()[PARAMETER_ID]));
        visitor.visit(mOutput1,                   static_cast<const corecvs::DoubleField *>(fields()[OUTPUT1_ID]));
        visitor.visit(mOutput2,                   static_cast<const corecvs::DoubleField *>(fields()[OUTPUT2_ID]));
    }

    AdderSubstractorParametersBase()
    {
        corecvs::DefaultSetter setter;
        accept(setter);
    }

    AdderSubstractorParametersBase(
          double input1
        , double input2
        , bool parameter
        , double output1
        , double output2
    )
    {
        mInput1 = input1;
        mInput2 = input2;
        mParameter = parameter;
        mOutput1 = output1;
        mOutput2 = output2;
    }

    friend std::ostream& operator << (std::ostream &out, AdderSubstractorParametersBase &toSave)
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
#endif  //ADDER_SUBSTRACTOR_PARAMETERS_BASE_H_
