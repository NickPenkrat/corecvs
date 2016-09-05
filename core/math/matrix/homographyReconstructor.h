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

#include "generated/homographyAlgorithm.h"
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

    void addPoint2PointConstraint  (const Vector2dd &from, const Vector2dd &to);
    void addPoint2LineConstraint   (const Vector2dd &from, const Line2d &line);
    void addPoint2SegmentConstraint(const Vector2dd &from, const Segment2d &line);

    void reset(void);

    bool hasEnoughtConstraints();

    double getCostFunction(Matrix33 &input, double out[] = NULL);
    int  getConstraintNumber();

    friend ostream & operator << (ostream &out, const HomographyReconstructor &reconstructor);

    /**
     *  This is a common entry point
     **/
    Matrix33 getBestHomography(const HomographyAlgorithm::HomographyAlgorithm &method = HomographyAlgorithm::ML_AFTER_LSE);

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

    /* Cost function related stuff */

    /**
     * This class reprents the function that stores
     * the model as a 3 by 3 matrix with lower left element fixed to zero
     *
     **/
    class CostFunction : public FunctionArgs {
    public:
        HomographyReconstructor *reconstructor;
        CostFunction(HomographyReconstructor *_reconstructor) : FunctionArgs(8, _reconstructor->getConstraintNumber()), reconstructor(_reconstructor) {}

        virtual void operator()(const double in[], double out[]);
    };

    /**
     * This class reprents the function that stores
     * the model as a meaningful decomposition of homography matrix
     *
     *  <ol>
     *    <li>First rotaton around current zero</li>
     *    <li>Then shift</li>
     *    <li>Uniform scale</li>
     *    <li>Projective transform</li>
     *  </ol>
     *
     *  \f[ H =
     *       \pmatrix{
     *             1  &  0   & 0\cr
     *             0  &  1   & 0\cr
     *            v_5 &  v_6 & 1
     *        }
     *       \pmatrix{
     *            v_4 &  0  & 0\cr
     *             0  & v_4 & 0\cr
     *             0  &  0  & 1
     *        }
     *       \pmatrix{
     *            1 & 0 & v_2\cr
     *            0 & 1 & v_3\cr
     *            0 & 0 & 1
     *        }
     *      \pmatrix{
     *         cos(v_1) & sin(v_1) & 0\cr
     *        -sin(v_1) & cos(v_1) & 0\cr
     *            0     &   0      & 1
     *        }
     *  \f]
     *
     **/
    class CostFunctionWize : public FunctionArgs {
    public:
        HomographyReconstructor *reconstructor;
        CostFunctionWize(HomographyReconstructor *_reconstructor) : FunctionArgs(6,_reconstructor->getConstraintNumber()), reconstructor(_reconstructor) {}

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

