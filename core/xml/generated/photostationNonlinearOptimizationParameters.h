#ifndef PHOTOSTATION_NONLINEAR_OPTIMIZATION_PARAMETERS_H_
#define PHOTOSTATION_NONLINEAR_OPTIMIZATION_PARAMETERS_H_
/**
 * \file photostationNonlinearOptimizationParameters.h
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
 * \brief Photostation Nonlinear Optimization Parameters 
 * Photostation Nonlinear Optimization Parameters 
 **/
class PhotostationNonlinearOptimizationParameters : public BaseReflection<PhotostationNonlinearOptimizationParameters>
{
public:
    enum FieldId {
        POSTAPPENDNONLINEARITERATIONS_ID,
        FINALNONLINEARITERATIONS_ID,
        ALTERNATINGITERATIONS_ID,
        EXCESSIVEQUATERNIONPARAMETRIZATION_ID,
        PHOTOSTATION_NONLINEAR_OPTIMIZATION_PARAMETERS_FIELD_ID_NUM
    };

    /** Section with variables */

    /** 
     * \brief postAppendNonlinearIterations 
     * Post-append non-linear optimization iterations 
     */
    int mPostAppendNonlinearIterations;

    /** 
     * \brief finalNonLinearIterations 
     * Final non-linear iterations 
     */
    int mFinalNonLinearIterations;

    /** 
     * \brief alternatingIterations 
     * Alternating optimization on success steps 
     */
    int mAlternatingIterations;

    /** 
     * \brief excessiveQuaternionParametrization 
     * Excessive/non-excessive quaternion parametrization 
     */
    bool mExcessiveQuaternionParametrization;

    /** Static fields init function, this is used for "dynamic" field initialization */ 
    static int staticInit();

    /** Section with getters */
    const void *getPtrById(int fieldId) const
    {
        return (const unsigned char *)(this) + fields()[fieldId]->offset;
    }
    int postAppendNonlinearIterations() const
    {
        return mPostAppendNonlinearIterations;
    }

    int finalNonLinearIterations() const
    {
        return mFinalNonLinearIterations;
    }

    int alternatingIterations() const
    {
        return mAlternatingIterations;
    }

    bool excessiveQuaternionParametrization() const
    {
        return mExcessiveQuaternionParametrization;
    }

    /* Section with setters */
    void setPostAppendNonlinearIterations(int postAppendNonlinearIterations)
    {
        mPostAppendNonlinearIterations = postAppendNonlinearIterations;
    }

    void setFinalNonLinearIterations(int finalNonLinearIterations)
    {
        mFinalNonLinearIterations = finalNonLinearIterations;
    }

    void setAlternatingIterations(int alternatingIterations)
    {
        mAlternatingIterations = alternatingIterations;
    }

    void setExcessiveQuaternionParametrization(bool excessiveQuaternionParametrization)
    {
        mExcessiveQuaternionParametrization = excessiveQuaternionParametrization;
    }

    /* Section with embedded classes */
    /* visitor pattern - http://en.wikipedia.org/wiki/Visitor_pattern */
template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(mPostAppendNonlinearIterations, static_cast<const IntField *>     (fields()[POSTAPPENDNONLINEARITERATIONS_ID]));
        visitor.visit(mFinalNonLinearIterations,  static_cast<const IntField *>     (fields()[FINALNONLINEARITERATIONS_ID]));
        visitor.visit(mAlternatingIterations,     static_cast<const IntField *>     (fields()[ALTERNATINGITERATIONS_ID]));
        visitor.visit(mExcessiveQuaternionParametrization, static_cast<const BoolField *>    (fields()[EXCESSIVEQUATERNIONPARAMETRIZATION_ID]));
    }

    PhotostationNonlinearOptimizationParameters()
    {
        DefaultSetter setter;
        accept(setter);
    }

    PhotostationNonlinearOptimizationParameters(
          int postAppendNonlinearIterations
        , int finalNonLinearIterations
        , int alternatingIterations
        , bool excessiveQuaternionParametrization
    )
    {
        mPostAppendNonlinearIterations = postAppendNonlinearIterations;
        mFinalNonLinearIterations = finalNonLinearIterations;
        mAlternatingIterations = alternatingIterations;
        mExcessiveQuaternionParametrization = excessiveQuaternionParametrization;
    }

    friend ostream& operator << (ostream &out, PhotostationNonlinearOptimizationParameters &toSave)
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
#endif  //PHOTOSTATION_NONLINEAR_OPTIMIZATION_PARAMETERS_H_