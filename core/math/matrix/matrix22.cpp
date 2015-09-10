#include "matrix22.h"

namespace corecvs {

/**
 *  Transposing the matrix
 **/
void Matrix22::transpose()
{
   double tmp;
   tmp = a(1,0); a(1,0) = a(0,1); a(0,1) = tmp;
}

/**
 *  Returning transposed matrix
 **/
Matrix22 Matrix22::transposed() const
{
   return Matrix22(
       a(0,0), a(1,0),
       a(0,1), a(1,1));
}

bool Matrix22::isInvertable(double tolerance)
{
    return det() > tolerance;
}


double Matrix22::det() const {
   return a00() * a11() - a01() * a10();
}

double Matrix22::trace() const
{
    return a00() + a11();
}

Vector2dd Matrix22::solve(const Matrix22 &A, const Vector2dd &b)
{
    double D = A.det();
    return Vector2dd (( A.a11() * b[0] - A.a01() * b[1]), (-A.a10() * b[0] + A.a00() * b[1])) / D;
}


Matrix22 operator* (const Matrix22 &m1, const Matrix22 &m2)
{
   return Matrix22(
           m1.a(0,0) * m2.a(0,0) + m1.a(0,1) * m2.a(1,0),
           m1.a(0,0) * m2.a(0,1) + m1.a(0,1) * m2.a(1,1),

           m1.a(1,0) * m2.a(0,0) + m1.a(1,1) * m2.a(1,0),
           m1.a(1,0) * m2.a(0,1) + m1.a(1,1) * m2.a(1,1)
   );
}


Matrix22 operator*= (Matrix22 &M1, const Matrix22 &M2)
{
    M1 = M1 * M2;
    return M1;
}

/**
 *  Get the i-th row vector
 **/
Vector2dd Matrix22::aV(int i) const
{
    return Vector2dd (a(i,0), a(i,1));
}

Vector2dd Matrix22::row(int i) const
{
    return aV(i);
}

/**
 *  Get the i-th column vector
 **/

Vector2dd Matrix22::aW(int i) const
{
    return Vector2dd (a(0,i), a(1,i));
}

Vector2dd Matrix22::column(int i) const
{
    return aW(i);
}



} // namespace corecvs

