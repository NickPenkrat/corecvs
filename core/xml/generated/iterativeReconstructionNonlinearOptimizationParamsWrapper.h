#ifndef ITERATIVE_RECONSTRUCTION_NONLINEAR_OPTIMIZATION_PARAMS_WRAPPER_H_
#define ITERATIVE_RECONSTRUCTION_NONLINEAR_OPTIMIZATION_PARAMS_WRAPPER_H_
/**
 * \file iterativeReconstructionNonlinearOptimizationParamsWrapper.h
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
#include "reconstructionFunctorOptimizationParams.h"

using namespace corecvs;

/*
 *  Additional includes for Pointer Types.
 */

namespace corecvs {
}
/*
 *  Additional includes for enum section.
 */
#include "reconstructionFunctorOptimizationErrorType.h"

/**
 * \brief Iterative Reconstruction Nonlinear Optimization Params Wrapper 
 * Iterative Reconstruction Nonlinear Optimization Params Wrapper 
 **/
class IterativeReconstructionNonlinearOptimizationParamsWrapper : public BaseReflection<IterativeReconstructionNonlinearOptimizationParamsWrapper>
{
public:
    enum FieldId {
        OPTIMIZATIONPARAMS_ID,
        ERRORTYPE_ID,
        POSTAPPENDNONLINEARITERATIONS_ID,
        FINALNONLINEARITERATIONS_ID,
        ALTERNATINGITERATIONS_ID,
        EXCESSIVEQUATERNIONPARAMETRIZATION_ID,
        EXPLICITINVERSE_ID,
        ITERATIVE_RECONSTRUCTION_NONLINEAR_OPTIMIZATION_PARAMS_WRAPPER_FIELD_ID_NUM
    };

    /** Section with variables */

    /** 
     * \brief optimizationParams 
     * optimizationParams 
     */
    ReconstructionFunctorOptimizationParams mOptimizationParams;

    /** 
     * \brief errorType 
     * Functor optimizattion error type 
     */
    int mErrorType;

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

    /** 
     * \brief explicitInverse 
     * Explisit inverse in Schur complement solving 
     */
    bool mExplicitInverse;

    /** Static fields init function, this is used for "dynamic" field initialization */ 
    static int staticInit();

    /** Section with getters */
    const void *getPtrById(int fieldId) const
    {
        return (const unsigned char *)(this) + fields()[fieldId]->offset;
    }
    ReconstructionFunctorOptimizationParams optimizationParams() const
    {
        return mOptimizationParams;
    }

    ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType errorType() const
    {
        return static_cast<ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType>(mErrorType);
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

    bool explicitInverse() const
    {
        return mExplicitInverse;
    }

    /* Section with setters */
    void setOptimizationParams(ReconstructionFunctorOptimizationParams const &optimizationParams)
    {
        mOptimizationParams = optimizationParams;
    }

    void setErrorType(ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType errorType)
    {
        mErrorType = errorType;
    }

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

    void setExplicitInverse(bool explicitInverse)
    {
        mExplicitInverse = explicitInverse;
    }

    /* Section with embedded classes */
    /* visitor pattern - http://en.wikipedia.org/wiki/Visitor_pattern */
template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(mOptimizationParams,        static_cast<const CompositeField *>(fields()[OPTIMIZATIONPARAMS_ID]));
        visitor.visit((int &)mErrorType,          static_cast<const EnumField *>    (fields()[ERRORTYPE_ID]));
        visitor.visit(mPostAppendNonlinearIterations, static_cast<const IntField *>     (fields()[POSTAPPENDNONLINEARITERATIONS_ID]));
        visitor.visit(mFinalNonLinearIterations,  static_cast<const IntField *>     (fields()[FINALNONLINEARITERATIONS_ID]));
        visitor.visit(mAlternatingIterations,     static_cast<const IntField *>     (fields()[ALTERNATINGITERATIONS_ID]));
        visitor.visit(mExcessiveQuaternionParametrization, static_cast<const BoolField *>    (fields()[EXCESSIVEQUATERNIONPARAMETRIZATION_ID]));
        visitor.visit(mExplicitInverse,           static_cast<const BoolField *>    (fields()[EXPLICITINVERSE_ID]));
    }

    IterativeReconstructionNonlinearOptimizationParamsWrapper()
    {
        DefaultSetter setter;
        accept(setter);
    }

    IterativeReconstructionNonlinearOptimizationParamsWrapper(
          ReconstructionFunctorOptimizationParams optimizationParams
        , ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType errorType
        , int postAppendNonlinearIterations
        , int finalNonLinearIterations
        , int alternatingIterations
        , bool excessiveQuaternionParametrization
        , bool explicitInverse
    )
    {
        mOptimizationParams = optimizationParams;
        mErrorType = errorType;
        mPostAppendNonlinearIterations = postAppendNonlinearIterations;
        mFinalNonLinearIterations = finalNonLinearIterations;
        mAlternatingIterations = alternatingIterations;
        mExcessiveQuaternionParametrization = excessiveQuaternionParametrization;
        mExplicitInverse = explicitInverse;
    }

    friend ostream& operator << (ostream &out, IterativeReconstructionNonlinearOptimizationParamsWrapper &toSave)
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
#endif  //ITERATIVE_RECONSTRUCTION_NONLINEAR_OPTIMIZATION_PARAMS_WRAPPER_H_
