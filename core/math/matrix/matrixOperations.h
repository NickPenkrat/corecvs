#ifndef MATRIXOPERATIONS_H_
#define MATRIXOPERATIONS_H_
/**
 * \file matrixOperations.h
 * \brief This header holds matrix operations that are common to
 * matrixes of different nature.
 *
 * \date Jul 4, 2011
 * \author alexander
 */

/**
 * \file matrixOperations.h
 * \brief This header holds matrix operations that are common to
 * matrixes of different nature.
 *
 * This class uses static polymorphism to access the elements of the container.
 * Two (or three) methods should be implemented to use MatrixOperationsBase
 *
 * ElementType &atm(int i, int j);
 * const ElementType &atm(int i, int j) const;
 * int height();
 * int width();
 * RealType createMatrix(int h, int w);
 *
 * Also this implementation expect that RealType has two defined subtype
 * InnerElementType
 *
 * This particular implementation doesn't expect that elements are continuous in memory
 *
 * This template class has 3 parameters
 *    ReturnType  - The type that should be returned in all constructive functions
 *    RealType    - The type that holds all the data needed for processing
 *    ElementType - The element type
 *
 * TODO: Consider using size_t instead of int
 * TODO: Consider using iterator and function object
 *
 * \date Aug 28, 2015
 **/

#include "global.h"
namespace corecvs {

template<typename RealType, typename ElementType, typename ReturnType = RealType>
class MatrixOperationsBase
{
public:
    /**
     *  Static cast functions
     **/
    inline RealType *realThis() {
        return static_cast<RealType *>(this);
    }

    inline const RealType *realThis() const {
        return static_cast<const RealType *>(this);
    }

private:
    /**
     *  Interface related functions
     **/
    inline ElementType &_atm(int i, int j) {
        return realThis()->atm(i, j);
    }

    inline const ElementType &_atm(int i, int j) const {
        return realThis()->atm(i, j);
    }

    inline int _height() const {
        return realThis()->height();
    }

    inline int _width() const {
        return realThis()->width();
    }

    inline RealType _createMatrix(int h, int w) const {
        return realThis()->createMatrix(h, w);
    }

public:

    friend ostream & operator <<(ostream &out, const RealType &matrix)
    {
        //streamsize wasPrecision = out.precision(6);
        for (int i = 0; i < _height(); i++)
        {
            out << "[";
            for (int j = 0; j < _width(); j++)
            {
                out << (j == 0 ? "" : " ");
                //out.width(6);
                out << matrix._atm(i,j);
            }
            out << "]\n";
        }
        //out.precision(wasPrecision);
        return out;
    }

    friend istream & operator >>(istream &in, RealType &matrix)
    {
        for (int i = 0; i < _height(); i++)
        {
            for (int j = 0; j < _width(); j++)
            {
                while (true)
                {
                    in >> matrix._atm(i,j);
                    // cout << "Read:" << i << "," << j << " " << matrix.a(i,j) << endl;
                    if (in.good())
                        break;
                    if (in.eof())
                        return in;
                    /* Not a number clean the error and advance*/
                    in.clear();
                    // cout << "Skipped:" << ((char)in.peek()) << endl;
                    in.ignore();
                }

            }
        }
        return in;
    }

    void transpose()
    {
        CORE_ASSERT_TRUE(_height() == _width(), "Matrix should be square to transpose.");

        for (int row = 0; row < _height(); row++)
        {
            for (int column = row + 1; column < _width(); column++)
            {
                ElementType tmp;
                tmp = _atm(column, row);
                _atm(column, row) = _atm(row, column);
                _atm(row, column) = tmp;
            }
        }
    }


    ReturnType operator -()
    {
        ReturnType result = _createMatrix(_height(), _width());
        for (int i = 0; i < result._height(); i++)
        {
            for (int j = 0; j < result._width(); j++)
            {
                result._atm(i, j) = -_atm(i,j);
            }
        }
        return result;
    }

    ReturnType operator *(const ElementType &a, const RealType &B)
    {
        ReturnType result = _createMatrix(B._height(), B._width());
        int row, column;
        for (row = 0; row < result._height(); row++)
        {
            for (column = 0; column < result._width(); column++)
            {
                 result._atm(row, column) = a *  B._atm(row, column);
            }
        }
        return result;
    }

    ReturnType operator *(const RealType &B, const ElementType &a)
    {
        ReturnType result = _createMatrix(B._height(), B._width());
        int row, column;
        for (row = 0; row < result._height(); row++)
        {
            for (column = 0; column < result._width(); column++)
            {
                 result._atm(row, column) = B._atm(row, column) * a;
            }
        }
        return result;
    }


    /* Work in progress. We should probably use vector operations for this */

    ReturnType operator +(const RealType &A, const RealType &B)
    {
        CORE_ASSERT_TRUE(A._width() == B._width() && A._height() == B._height(), "Matrices have wrong sizes");
        ReturnType result = _createMatrix(A._height(), A._width());

        for (row = 0; row < result._height(); row++)
        {
            for (column = 0; column < result._width(); column++)
            {
                result._atm(row, column) = A._atm(row, column) + B._atm(row, column);
            }
        }

        return result;
    }


    ReturnType operator -(const RealType &A, const RealType &B)
    {
        CORE_ASSERT_TRUE(A._width() == B._width() && A._height() == B._height(), "Matrices have wrong sizes");
        ReturnType result = _createMatrix(A._height(), A._width());

        for (row = 0; row < result._height(); row++)
        {
            for (column = 0; column < result._width(); column++)
            {
                result._atm(row, column) = A._atm(row, column) - B._atm(row, column);
            }
        }

        return result;
    }


    ReturnType Matrix::mul(const RealType& V)
    {
        CORE_ASSERT_TRUE(this->w == V.h, "Matrices have wrong sizes");
        Matrix *result = new Matrix(this->h, V.w, false);

        int row, column, runner;
        for (row = 0; row < result->h; row++)
            for (column = 0; column < result->w; column++)
            {
                double sum = 0;
                for (runner = 0; runner < this->w; runner++)
                {
                    sum += this->a(row, runner) * V.a(runner, column);
                }
                result->a(row, column) = sum;
            }

        return result;
    }


};


} //namespace corecvs
#endif  //MATRIXOPERATIONS_H_



