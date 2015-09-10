#ifndef MATRIX22_H_
#define MATRIX22_H_

/**
 * \file matrix22.h
 * \brief This file holds the header for the commonly used structure -
 * a 2 by 2 matrix.
 *
 * \f[ P=\pmatrix{
 *        a_{1,1} & a_{1,2} \cr
 *        a_{2,1} & a_{2,2} }
 * \f]
 *
 * \ingroup cppcorefiles
 * \date Aug 31, 2015
 **/

#include <iostream>
#include <limits>

#include <math.h>
#include <string.h>

#include "vector3d.h"
#include "fixedVector.h"
namespace corecvs {

using std::numeric_limits;
using std::streamsize;

/**
 *  Class describes the 2 by 2 matrix.
 *
 * \f[ P=\pmatrix{
 *        a_{1,1} & a_{1,2} \cr
 *        a_{2,1} & a_{2,2} }
 * \f]
 *
 *
 **/

class  Matrix22 : public FixedVectorBase<Matrix22, double, 4>
{
public:
    typedef FixedVectorBase<Matrix22, double, 4> BaseClass;

    static const int H = 2;
    static const int W = H;
    static const int ELEM_NUM = H * W;


    Matrix22() {}

    /**
     * Constructs the 2 by 2 Matrix with the given diagonal element
     * In terms of projective transform that will always be an identity matrix
     *
     * \f[ P=\pmatrix{
     *        d & 0 \cr
     *        0 & d  }
     * \f]
     **/
    explicit Matrix22(double d) : BaseClass(0.0)
    {
        for (int i = 0; i < H; i++)
            a(i,i) = d;
    }

    Matrix22(const FixedVector<double, 4> &data) : BaseClass(data)
    { }

    Matrix22(double _a00, double _a01,
             double _a10, double _a11
             );

    /* Element accessors */
    double &a(int i,int j);
    const double &a(int i,int j) const;

    double &operator ()(int i,int j);
    const double &operator ()(int i,int j) const;

    /* Matrix is so small that for convenience we could provide an accessor for all elements*/
    double &a00();
    double &a01();
    double &a10();
    double &a11();
    const double &a00() const;
    const double &a01() const;
    const double &a10() const;
    const double &a11() const;


    void transpose();
    Matrix22 transposed() const;

    bool isInvertable(double tolerance = 1e-9);
    Matrix22 inverted() const;
    double det() const;
    double trace() const;

    /**
     *  This method solves the system
     *  Ax=b by Kramer method
     **/
    static Vector2dd solve(const Matrix22 &A, const Vector2dd &b);

    friend Matrix22 operator * (const Matrix22 &M1, const Matrix22 &M2);
    friend Matrix22 operator *=(      Matrix22 &M1, const Matrix22 &M2);

    friend inline Vector2dd operator *(const Matrix22 &matrix, const Vector2dd &V);
    friend inline Vector2dd operator *(const Vector2dd &V, const Matrix22 &matrix);


    Vector2dd     aV(int i) const;
    Vector2dd    row(int i) const;
    Vector2dd     aW(int i) const;
    Vector2dd column(int i) const;
};

/**
 *  Geting Matrix element functions block
 **/

inline double &Matrix22::a(int i,int j)
{
    return (*this)[i * W + j];
}

inline const double &Matrix22::a(int i,int j) const
{
    return (*this)[i * W + j];
}

inline double &Matrix22::operator ()(int i,int j)
{
    return (*this)[i * W + j];
}

inline const double &Matrix22::operator ()(int i,int j) const
{
    return (*this)[i * W + j];
}

double &Matrix22::a00()
{
    return (*this)[0];
}

double &Matrix22::a01()
{
    return (*this)[1];
}

double &Matrix22::a10()
{
    return (*this)[2];
}

double &Matrix22::a11()
{
    return (*this)[3];
}

const double &Matrix22::a00() const
{
    return (*this)[0];
}

const double &Matrix22::a01() const
{
    return (*this)[1];
}

const double &Matrix22::a10() const
{
    return (*this)[2];
}

const double &Matrix22::a11() const
{
    return (*this)[3];
}


} //namespace  corecvs

#endif // MATRIX22_H_
