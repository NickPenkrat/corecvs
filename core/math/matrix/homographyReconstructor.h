#ifndef HOMOGRAPHYRECONSTRUCTOR_H_
#define HOMOGRAPHYRECONSTRUCTOR_H_
/**
 * \file homographyReconstructor.h
 * \brief Add Comment Here
 *
 * \date Jun 26, 2011
 * \author alexander
 */
#include <iostream>
#include <vector>

#include "vector2d.h"
#include "matrix33.h"
#include "polygons.h"
#include "correspondenceList.h"
#include "matrix.h"
#include "function.h"
#include "line.h"
namespace corecvs {

using std::vector;

typedef PrimitiveCorrespondence<Vector2dd, Line2d> CorrespondencePointLine;
typedef PrimitiveCorrespondence<Vector2dd, Segment2d> CorrespondencePointSegment;


class HomographyReconstructor
{
public:
    vector<Correspondence> p2p;
    vector<CorrespondencePointLine> p2l;
    vector<CorrespondencePointSegment> p2s;

public:
    bool trace = false;

    HomographyReconstructor();

    void addPoint2PointConstraint(const Vector2dd &from, const Vector2dd &to);
    void addPoint2LineConstraint(const Vector2dd &from, const Line2d &line);
    void addPoint2SegmentConstraint(const Vector2dd &from, const Segment2d &line);

    void reset(void);

    bool hasEnoughtConstraints();

    double getCostFunction(Matrix33 &input, double out[] = NULL);
    int  getConstraintNumber();

    friend ostream & operator << (ostream &out, const HomographyReconstructor &reconstructor);

    enum HomographyAlgorithm {
        LSE,
        LSE1,
        LSE2,
        ML,
        ML_AFTER_LSE,
        LAST
    };

    static inline const char *getName(const HomographyAlgorithm &value)
    {
        switch (value)
        {
            case LSE : return "LSE"; break ;
            case LSE1 : return "LSE1"; break ;
            case LSE2 : return "LSE2"; break ;
            case ML : return "ML"; break ;
            case ML_AFTER_LSE : return "ML_AFTER_LSE"; break ;
            default : return "Not in range"; break ;
        }
        return "Not in range";
    }

    /**
     *  This is a common entry point
     **/
    Matrix33 getBestHomography(const HomographyAlgorithm &method = ML_AFTER_LSE);

    /**
     *  Function was introduced for testing. Usage is discouraged. Use getBestHomographyLSE1
     **/
    Matrix33 getBestHomographyLSE(void);

    /**
     *  This method uses SVD of (A^T A) matrix to solve
     **/
    Matrix33 getBestHomographyLSE1(void);
    /**
     *  This method uses inverse SVD to solve
     **/
    Matrix33 getBestHomographyLSE2(void);


    Matrix33 getBestHomographyLM(Matrix33 guess = Matrix33(1.0));
    Matrix33 getBestHomographyLSEandLM();


    void normalisePoints(Matrix33 &transformLeft, Matrix33 &transformRight);

    virtual ~HomographyReconstructor();

private:
    void addPoint2PointConstraintLSE(
            Matrix &A,
            Matrix &B,
            int num,
            const Vector2dd &from, const Vector2dd &to);

    void addPoint2LineConstraintLSE(
            Matrix &A,
            Matrix &B,
            int num,
            const Vector2dd &from, const Line2d &line);

    void addPoint2LineConstraintLSEUnif(
            Matrix &A,
            int num,
            const Vector2dd &from, const Line2d &line);

    void addPoint2PointConstraintLSEUnif(
            Matrix &A,
            int num,
            const Vector2dd &from, const Vector2dd &to);

    class CostFunction : public FunctionArgs {
    public:
        HomographyReconstructor *reconstructor;
        CostFunction(HomographyReconstructor *_reconstructor) : FunctionArgs(8, _reconstructor->getConstraintNumber()), reconstructor(_reconstructor) {}

        virtual void operator()(const double in[], double out[]);
    };

    class CostFunctionWize : public FunctionArgs {
    public:
        HomographyReconstructor *reconstructor;
        CostFunctionWize(HomographyReconstructor *_reconstructor) : FunctionArgs(8,_reconstructor->getConstraintNumber()), reconstructor(_reconstructor) {}

        virtual void operator()(const double in[], double out[]);
        Matrix33 matrixFromState(const double in[]);
    };




#if 0
    /**
     * \deprecated
     **/
    Matrix33 getBestHomographyClassicKalman(void);
    /**
     * \deprecated
     **/
    Matrix33 getBestHomographyFastKalman(void);
#endif
};


} //namespace corecvs
#endif /* HOMOGRAPHYRECONSTRUCTOR_H_ */

