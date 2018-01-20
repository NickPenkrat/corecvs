#ifndef CATADIOPTRIC_BASE_PARAMETERS_H_
#define CATADIOPTRIC_BASE_PARAMETERS_H_
/**
 * \file catadioptricBaseParameters.h
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 * Generated from projections.xml
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
 * \brief Catadioptric Base Parameters 
 * Catadioptric Base Parameters 
 **/
class CatadioptricBaseParameters : public corecvs::BaseReflection<CatadioptricBaseParameters>
{
public:
    enum FieldId {
        PRINCIPALX_ID,
        PRINCIPALY_ID,
        FOCAL_ID,
        N_ID,
        SIZEX_ID,
        SIZEY_ID,
        DISTORTEDSIZEX_ID,
        DISTORTEDSIZEY_ID,
        CATADIOPTRIC_BASE_PARAMETERS_FIELD_ID_NUM
    };

    /** Section with variables */

    /** 
     * \brief principalX 
     * The center of projection \f$x_c\f$ 
     */
    double mPrincipalX;

    /** 
     * \brief principalY 
     * The center of projection \f$y_c\f$ 
     */
    double mPrincipalY;

    /** 
     * \brief focal 
     * focal 
     */
    double mFocal;

    /** 
     * \brief n 
     * n 
     */
    vector<double> mN;

    /** 
     * \brief sizeX 
     * Model image resolution X 
     */
    double mSizeX;

    /** 
     * \brief sizeY 
     * Model image resolution Y 
     */
    double mSizeY;

    /** 
     * \brief distortedSizeX 
     * Source image resolution X 
     */
    double mDistortedSizeX;

    /** 
     * \brief distortedSizeY 
     * Source image resolution Y 
     */
    double mDistortedSizeY;

    /** Static fields init function, this is used for "dynamic" field initialization */ 
    static int staticInit();

    static int relinkCompositeFields();

    /** Section with getters */
    const void *getPtrById(int fieldId) const
    {
        return (const unsigned char *)(this) + fields()[fieldId]->offset;
    }
    double principalX() const
    {
        return mPrincipalX;
    }

    double principalY() const
    {
        return mPrincipalY;
    }

    double focal() const
    {
        return mFocal;
    }

    vector<double> n() const
    {
        return mN;
    }

    double sizeX() const
    {
        return mSizeX;
    }

    double sizeY() const
    {
        return mSizeY;
    }

    double distortedSizeX() const
    {
        return mDistortedSizeX;
    }

    double distortedSizeY() const
    {
        return mDistortedSizeY;
    }

    /* Section with setters */
    void setPrincipalX(double principalX)
    {
        mPrincipalX = principalX;
    }

    void setPrincipalY(double principalY)
    {
        mPrincipalY = principalY;
    }

    void setFocal(double focal)
    {
        mFocal = focal;
    }

    void setN(vector<double> n)
    {
        mN = n;
    }

    void setSizeX(double sizeX)
    {
        mSizeX = sizeX;
    }

    void setSizeY(double sizeY)
    {
        mSizeY = sizeY;
    }

    void setDistortedSizeX(double distortedSizeX)
    {
        mDistortedSizeX = distortedSizeX;
    }

    void setDistortedSizeY(double distortedSizeY)
    {
        mDistortedSizeY = distortedSizeY;
    }

    /* Section with embedded classes */
    /* visitor pattern - http://en.wikipedia.org/wiki/Visitor_pattern */
template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(mPrincipalX,                static_cast<const corecvs::DoubleField *>(fields()[PRINCIPALX_ID]));
        visitor.visit(mPrincipalY,                static_cast<const corecvs::DoubleField *>(fields()[PRINCIPALY_ID]));
        visitor.visit(mFocal,                     static_cast<const corecvs::DoubleField *>(fields()[FOCAL_ID]));
        visitor.visit(mN,                         static_cast<const corecvs::DoubleVectorField *>(fields()[N_ID]));
        visitor.visit(mSizeX,                     static_cast<const corecvs::DoubleField *>(fields()[SIZEX_ID]));
        visitor.visit(mSizeY,                     static_cast<const corecvs::DoubleField *>(fields()[SIZEY_ID]));
        visitor.visit(mDistortedSizeX,            static_cast<const corecvs::DoubleField *>(fields()[DISTORTEDSIZEX_ID]));
        visitor.visit(mDistortedSizeY,            static_cast<const corecvs::DoubleField *>(fields()[DISTORTEDSIZEY_ID]));
    }

    CatadioptricBaseParameters()
    {
        corecvs::DefaultSetter setter;
        accept(setter);
    }

    CatadioptricBaseParameters(
          double principalX
        , double principalY
        , double focal
        , vector<double> n
        , double sizeX
        , double sizeY
        , double distortedSizeX
        , double distortedSizeY
    )
    {
        mPrincipalX = principalX;
        mPrincipalY = principalY;
        mFocal = focal;
        mN = n;
        mSizeX = sizeX;
        mSizeY = sizeY;
        mDistortedSizeX = distortedSizeX;
        mDistortedSizeY = distortedSizeY;
    }

    bool operator ==(const CatadioptricBaseParameters &other) const 
    {
        if ( !(this->mPrincipalX == other.mPrincipalX)) return false;
        if ( !(this->mPrincipalY == other.mPrincipalY)) return false;
        if ( !(this->mFocal == other.mFocal)) return false;
        if ( !(this->mN == other.mN)) return false;
        if ( !(this->mSizeX == other.mSizeX)) return false;
        if ( !(this->mSizeY == other.mSizeY)) return false;
        if ( !(this->mDistortedSizeX == other.mDistortedSizeX)) return false;
        if ( !(this->mDistortedSizeY == other.mDistortedSizeY)) return false;
        return true;
    }
    friend std::ostream& operator << (std::ostream &out, CatadioptricBaseParameters &toSave)
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
#endif  //CATADIOPTRIC_BASE_PARAMETERS_H_
