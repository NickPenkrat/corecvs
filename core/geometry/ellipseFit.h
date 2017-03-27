#ifndef ELLIPSEFIT_H
#define ELLIPSEFIT_H

#include "ellipse.h"


namespace corecvs {

/**
 *  http://cseweb.ucsd.edu/~mdailey/Face-Coord/ellipse-specific-fitting.pdf
 *
 **/
class EllipseFit
{
public:
    EllipseFit();
    vector<Vector2dd> points;
    void addPoint(Vector2dd &point);

    /** Simplest way that involves only in'da'house methods and uses ellipse resctriction **/
    SecondOrderCurve solveDummy();

#ifdef WITH_BLAS
    SecondOrderCurve solveBLAS();
#endif


    /** Generic Quadric without ellipse restrictions **/
    SecondOrderCurve solveQuadricDummy();


};

} // namespace corecvs

#endif // ELLIPSEFIT_H
