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
#include "correspondanceList.h"
#include "matrix.h"
#include "function.h"
#include "line.h"
namespace core3vi {

using std::vector;

typedef PrimitiveCorrespondance<Vector2dd, Line2d> CorrespondancePointLine;
typedef PrimitiveCorrespondance<Vector2dd, Segment2d> CorrespondancePointSegment;


class HomographyReconstructor
{
public:
    vector<Correspondance> p2p;
    vector<CorrespondancePointLine> p2l;
    vector<CorrespondancePointSegment> p2s;


    HomographyReconstructor();

    void addPoint2PointConstraint(const Vector2dd &from, const Vector2dd &to);
    void addPoint2LineConstraint(const Vector2dd &from, const Line2d &line);
    void addPoint2SegmentConstraint(const Vector2dd &from, const Segment2d &line);

    void reset(void);

    bool hasEnoughtConstraints();

    double getCostFunction(Matrix33 &input);

    friend ostream & operator << (ostream &out, const HomographyReconstructor &reconstructor);

    Matrix33 getBestHomography(int method);

    Matrix33 getBestHomographyLSE(void);
    Matrix33 getBestHomographyLSE1(void);
    Matrix33 getBestHomographyLSE2(void);
    Matrix33 getBestHomographyClassicKalman(void);
    Matrix33 getBestHomographyFastKalman(void);
    Matrix33 getBestHomographyLM(Matrix33 guess = Matrix33(1.0));


    void noramlisePoints(Matrix33 &transformLeft, Matrix33 &transformRight);

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
        CostFunction(HomographyReconstructor *_reconstructor) : FunctionArgs(8,1), reconstructor(_reconstructor) {};

        virtual void operator()(const double in[], double out[]);
    };

    class CostFunctionWize : public FunctionArgs {
    public:
        HomographyReconstructor *reconstructor;
        CostFunctionWize(HomographyReconstructor *_reconstructor) : FunctionArgs(8,1), reconstructor(_reconstructor) {};

        virtual void operator()(const double in[], double out[]);
        Matrix33 matrixFromState(const double in[]);
    };


};


} //namespace core3vi
#endif /* HOMOGRAPHYRECONSTRUCTOR_H_ */

