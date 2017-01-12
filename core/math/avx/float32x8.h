#ifndef FLOAT32X8_H_
#define FLOAT32X8_H_
/**
 * \file float32x4.h
 * \brief a header for Float32x4.c
 *
 * \ingroup cppcorefiles
 * \date Sep 25, 2010
 * \author: alexander
 */

#include "global.h"

namespace corecvs {

class ALIGN_DATA(16) Float32x8
{
public:
     static const int SIZE = 8;

    __m256 data;



    /* Constructors */
    Float32x8() {}

    /**
    *  Copy constructor
    **/
    Float32x8(const Float32x8 &other) {
        this->data = other.data;
    }

    /**
    *  Constructor from intrinsic type
    **/
    explicit Float32x8(const __m256 &_data) {
        this->data = _data;
    }

    /**
    *  Constructor from integer type
    **/
    explicit Float32x8(const Int32x8 &other) {
        this->data = _mm256_cvtepi32_ps(other.data);
    }

    /**
    *  Fills the vector with 4 same float values
    **/
    explicit Float32x8(const float value) {
        CORE_ASSERT_TRUE(sizeof(float) == 4, "Float should be 4 byte")
        this->data = _mm256_set1_ps(value);
    }

    /**
     *  Extracts the square root from vector elements
     *  \f[
     *    \begin{array}{cccc}
     *         a_0    &    a_1    &    a_2    &    a_3    \\ \hline
     *      \sqrt a_0 & \sqrt a_1 & \sqrt a_2 & \sqrt a_3
     *    \end{array}
     *  \f]
     **/
    inline Float32x8 sqrt() const
    {
        return Float32x8(_mm256_sqrt_ps(this->data));
    }

    /**
     *  truncates vector elements
     *  \f[
     *    \begin{array}{cccc}
     *        a_0   &   a_1   &   a_2   &   a_3   \\ \hline
     *      [ a_0 ] & [ a_1 ] & [ a_2 ] & [ a_3 ]
     *    \end{array}
     *  \f]
     **/
    inline Int32x8 trunc() const
    {
        return Int32x8(_mm256_cvttps_epi32(this->data));
    }

    /* Arithmetics operations */
    friend Float32x8 operator +(const Float32x8 &left, const Float32x8 &right);
    friend Float32x8 operator -(const Float32x8 &left, const Float32x8 &right);

    friend Float32x8 operator +=(Float32x8 &left, const Float32x8 &right);
    friend Float32x8 operator -=(Float32x8 &left, const Float32x8 &right);

    friend Float32x8 operator *(const Float32x8 &left, const Float32x8 &right);
    friend Float32x8 operator /(const Float32x8 &left, const Float32x8 &right);

    friend Float32x8 operator *=(Float32x8 &left, const Float32x8 &right);
    friend Float32x8 operator /=(Float32x8 &left, const Float32x8 &right);

};


FORCE_INLINE Float32x8 operator +(const Float32x8 &left, const Float32x8 &right) {
    return Float32x8(_mm256_add_ps(left.data, right.data));
}

FORCE_INLINE Float32x8 operator -(const Float32x8 &left, const Float32x8 &right) {
    return Float32x8(_mm256_sub_ps(left.data, right.data));
}

FORCE_INLINE Float32x8 operator +=(Float32x8 &left, const Float32x8 &right) {
    left.data = _mm256_add_ps(left.data, right.data);
    return left;
}

FORCE_INLINE Float32x8 operator -=(Float32x8 &left, const Float32x8 &right) {
    left.data = _mm256_sub_ps(left.data, right.data);
    return left;
}


FORCE_INLINE Float32x8 operator *(const Float32x8 &left, const Float32x8 &right) {
    return Float32x8(_mm256_mul_ps(left.data, right.data));
}

FORCE_INLINE Float32x8 operator /(const Float32x8 &left, const Float32x8 &right) {
    return Float32x8(_mm256_div_ps(left.data, right.data));
}

FORCE_INLINE Float32x8 operator *=(Float32x8 &left, const Float32x8 &right) {
    left.data = _mm256_mul_ps(left.data, right.data);
    return left;
}

FORCE_INLINE Float32x8 operator /=(Float32x8 &left, const Float32x8 &right) {
    left.data = _mm256_div_ps(left.data, right.data);
    return left;
}

/* Some functions working with integer operations */

FORCE_INLINE Int32x8 operator /     (const Int32x8 &left, float divisor) {
    Float32x8 invDivisor(1.0f / divisor);
    return (Float32x8(left) * invDivisor).trunc();
}

FORCE_INLINE Int32x8 operator /     (const Int32x8 &left, int divisor) {
    return operator / (left, (float) divisor);
}

FORCE_INLINE Int32x8 operator /=    (Int32x8 &left, float divisor) {
    left = operator / (left, divisor);
    return left;
}

FORCE_INLINE Int32x8 operator /=    (Int32x8 &left, int divisor) {
    return operator /= (left, (float) divisor);
}


} //namespace corecvs

#endif  //FLOAT32X8_H_