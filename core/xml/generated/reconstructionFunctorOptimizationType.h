#ifndef RECONSTRUCTION_FUNCTOR_OPTIMIZATION_TYPE_H_
#define RECONSTRUCTION_FUNCTOR_OPTIMIZATION_TYPE_H_
/**
 * \file reconstructionFunctorOptimizationType.h
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
 * \brief Reconstruction Functor Optimization Type 
 * Reconstruction Functor Optimization Type 
 **/
class ReconstructionFunctorOptimizationType : public BaseReflection<ReconstructionFunctorOptimizationType>
{
public:
    enum FieldId {
        RECONSTRUCTION_FUNCTOR_OPTIMIZATION_TYPE_FIELD_ID_NUM
    };

    /** Section with variables */

    /** Static fields init function, this is used for "dynamic" field initialization */ 
    static int staticInit();

    /** Section with getters */
    const void *getPtrById(int fieldId) const
    {
        return (const unsigned char *)(this) + fields()[fieldId]->offset;
    }
    /* Section with setters */
    /* Section with embedded classes */
    /* visitor pattern - http://en.wikipedia.org/wiki/Visitor_pattern */
template<class VisitorType>
    void accept(VisitorType &visitor)
    {
    }

    ReconstructionFunctorOptimizationType()
    {
        DefaultSetter setter;
        accept(setter);
    }

    ReconstructionFunctorOptimizationType(
    )
    {
    }

    friend ostream& operator << (ostream &out, ReconstructionFunctorOptimizationType &toSave)
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
#endif  //RECONSTRUCTION_FUNCTOR_OPTIMIZATION_TYPE_H_