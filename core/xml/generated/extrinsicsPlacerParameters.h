#ifndef EXTRINSICS_PLACER_PARAMETERS_H_
#define EXTRINSICS_PLACER_PARAMETERS_H_
/**
 * \file extrinsicsPlacerParameters.h
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 * Generated from calibration.xml
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
 * \brief Extrinsics Placer Parameters 
 * Extrinsics Placer Parameters 
 **/
class ExtrinsicsPlacerParameters : public corecvs::BaseReflection<ExtrinsicsPlacerParameters>
{
public:
    enum FieldId {
        TRIANGULATE_ON_SPHERE_ID,
        SKYDOME_SIZE_ID,
        ITERATIONS_ID,
        LOCK_1_CAM_ID,
        LOCK_ORIENTATIONS_ID,
        LOCK_POSITIONS_ID,
        EXTRINSICS_PLACER_PARAMETERS_FIELD_ID_NUM
    };

    /** Section with variables */

    /** 
     * \brief triangulate on sphere 
     * triangulate on sphere 
     */
    bool mTriangulateOnSphere;

    /** 
     * \brief Skydome size 
     * Skydome size 
     */
    double mSkydomeSize;

    /** 
     * \brief Iterations 
     * Iterations 
     */
    int mIterations;

    /** 
     * \brief lock 1 cam 
     * lock 1 cam 
     */
    bool mLock1Cam;

    /** 
     * \brief lock orientations 
     * lock orientations 
     */
    bool mLockOrientations;

    /** 
     * \brief lock positions 
     * lock positions 
     */
    bool mLockPositions;

    /** Static fields init function, this is used for "dynamic" field initialization */ 
    static int staticInit();

    static int relinkCompositeFields();

    /** Section with getters */
    const void *getPtrById(int fieldId) const
    {
        return (const unsigned char *)(this) + fields()[fieldId]->offset;
    }
    bool triangulateOnSphere() const
    {
        return mTriangulateOnSphere;
    }

    double skydomeSize() const
    {
        return mSkydomeSize;
    }

    int iterations() const
    {
        return mIterations;
    }

    bool lock1Cam() const
    {
        return mLock1Cam;
    }

    bool lockOrientations() const
    {
        return mLockOrientations;
    }

    bool lockPositions() const
    {
        return mLockPositions;
    }

    /* Section with setters */
    void setTriangulateOnSphere(bool triangulateOnSphere)
    {
        mTriangulateOnSphere = triangulateOnSphere;
    }

    void setSkydomeSize(double skydomeSize)
    {
        mSkydomeSize = skydomeSize;
    }

    void setIterations(int iterations)
    {
        mIterations = iterations;
    }

    void setLock1Cam(bool lock1Cam)
    {
        mLock1Cam = lock1Cam;
    }

    void setLockOrientations(bool lockOrientations)
    {
        mLockOrientations = lockOrientations;
    }

    void setLockPositions(bool lockPositions)
    {
        mLockPositions = lockPositions;
    }

    /* Section with embedded classes */
    /* visitor pattern - http://en.wikipedia.org/wiki/Visitor_pattern */
template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(mTriangulateOnSphere,       static_cast<const corecvs::BoolField *>(fields()[TRIANGULATE_ON_SPHERE_ID]));
        visitor.visit(mSkydomeSize,               static_cast<const corecvs::DoubleField *>(fields()[SKYDOME_SIZE_ID]));
        visitor.visit(mIterations,                static_cast<const corecvs::IntField *>(fields()[ITERATIONS_ID]));
        visitor.visit(mLock1Cam,                  static_cast<const corecvs::BoolField *>(fields()[LOCK_1_CAM_ID]));
        visitor.visit(mLockOrientations,          static_cast<const corecvs::BoolField *>(fields()[LOCK_ORIENTATIONS_ID]));
        visitor.visit(mLockPositions,             static_cast<const corecvs::BoolField *>(fields()[LOCK_POSITIONS_ID]));
    }

    ExtrinsicsPlacerParameters()
    {
        corecvs::DefaultSetter setter;
        accept(setter);
    }

    ExtrinsicsPlacerParameters(
          bool triangulateOnSphere
        , double skydomeSize
        , int iterations
        , bool lock1Cam
        , bool lockOrientations
        , bool lockPositions
    )
    {
        mTriangulateOnSphere = triangulateOnSphere;
        mSkydomeSize = skydomeSize;
        mIterations = iterations;
        mLock1Cam = lock1Cam;
        mLockOrientations = lockOrientations;
        mLockPositions = lockPositions;
    }

    bool operator ==(const ExtrinsicsPlacerParameters &other) const 
    {
        if ( !(this->mTriangulateOnSphere == other.mTriangulateOnSphere)) return false;
        if ( !(this->mSkydomeSize == other.mSkydomeSize)) return false;
        if ( !(this->mIterations == other.mIterations)) return false;
        if ( !(this->mLock1Cam == other.mLock1Cam)) return false;
        if ( !(this->mLockOrientations == other.mLockOrientations)) return false;
        if ( !(this->mLockPositions == other.mLockPositions)) return false;
        return true;
    }
    friend std::ostream& operator << (std::ostream &out, ExtrinsicsPlacerParameters &toSave)
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
#endif  //EXTRINSICS_PLACER_PARAMETERS_H_
