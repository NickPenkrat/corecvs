#ifndef POLYNOMIAL_H
#define POLYNOMIAL_H

#include "vector.h"
#include "abstractBuffer.h"
#include "matrix.h"

#include <vector>

namespace corecvs
{
/*
 * This class stores polynomials.
 * \alpha_0+\alpha_1 x + \dots + \alpha_n x^n
 * is stored as \alpha_0 \dots \alpha_n
 */
class Polynomial : protected std::vector<double>
{
public:
    double&  operator[](const size_t &power);
    const double   operator[](const size_t &power) const;
    const double* data() const;
    /*
     * Horner-scheme evaluator
     * Note, that this is not machine-accurate evaluator
     * TODO: replace with machine-accurate evaluator (for example -- Luc Paquet Precise evaluation of a polynomial at a point given in staggered
     * correction format [Journal of Computational and Applied Mathematics 50 (1994)]
     */
    double         operator()(const double &at   ) const;

    // TODO: This stuff should be out-of-class
    Polynomial  operator* (const double &v) const;
    Polynomial& operator*=(const double &v);
    Polynomial  operator* (const Polynomial &rhs) const;
    Polynomial& operator*=(const Polynomial &rhs);
    Polynomial  operator+ (const Polynomial &rhs) const;
    Polynomial& operator+=(const Polynomial &rhs);
    Polynomial  operator- (const Polynomial &rhs) const;
    Polynomial& operator-=(const Polynomial &rhs);


    size_t  degree() const;
    
    Polynomial(const double &x0 = 0.0);
    Polynomial(const double *coeff, size_t degree);
    Polynomial(const corecvs::Vector &coeff);
    Polynomial(const std::vector<double> &coeff);

    static Polynomial X();
    static Polynomial FromRoots(const std::vector<double> &roots);
    static Polynomial Interpolate(const std::vector<double> &x, const std::vector<double> &Px);

    template<typename V>
    void accept(V& visitor)
    {
        visitor.visit((std::vector<double>&)*this, "coeff");
    }
};

class PolynomialMatrix : public corecvs::AbstractBuffer<Polynomial, int32_t>
{
public:
    PolynomialMatrix(int h = 0, int w = 0, const Polynomial& poly = 0.0);
    corecvs::Matrix operator() (const double &x) const;
    Polynomial det(const size_t requiredPower) const;
};
}



std::ostream& operator<<(std::ostream&, const corecvs::Polynomial& p);


#endif
