#include "polynomial.h"

#include <iomanip>

const double* corecvs::Polynomial::data() const
{
    return &static_cast<const std::vector<double>&>(*this)[0];
}

double& corecvs::Polynomial::operator[] (const size_t &power)
{
    if (power >= size())
        resize(power + 1);
    return static_cast<std::vector<double>&>(*this)[power];
}

const double corecvs::Polynomial::operator[] (const size_t &power) const
{
    return power >= size() ? 0 : static_cast<const std::vector<double>&>(*this)[power];
}

double  corecvs::Polynomial::operator() (const double &at) const
{
    size_t deg = degree();
    if (!size())
        return 0.0;
    double v = (*this)[deg];
    for (size_t i = deg; i > 0; --i)
    {
        v = (*this)[i - 1] + v * at;
    }
    return v;
}

size_t corecvs::Polynomial::degree() const
{
    return size() ? size() - 1 : 0;
}

corecvs::Polynomial corecvs::Polynomial::operator* (const double &v) const
{
    corecvs::Polynomial ans = *this;
    ans *= v;
    return ans;
}

corecvs::Polynomial& corecvs::Polynomial::operator*=(const double &v)
{
    for (auto& p: *this)
        p *= v;
    return *this;
}

corecvs::Polynomial  corecvs::Polynomial::operator* (const corecvs::Polynomial &rhs) const
{
    corecvs::Polynomial ans = *this;
    ans *= rhs;
    return ans;
}

corecvs::Polynomial& corecvs::Polynomial::operator*=(const corecvs::Polynomial &rhs)
{
    size_t degl = degree(), degr = rhs.degree();
    size_t total = degl + degr;
    std::vector<double> coeff(total + 1);
    for (size_t i = 0; i <= degl; ++i)
    {
        for (size_t j = 0; j <= degr; ++j)
        {
            coeff[i + j] += (*this)[i] * rhs[j];
        }
    }
    *this = coeff;
    return *this;
}

corecvs::Polynomial  corecvs::Polynomial::operator+ (const corecvs::Polynomial &rhs) const
{
    corecvs::Polynomial ans = *this;
    ans += rhs;
    return ans;
}

corecvs::Polynomial& corecvs::Polynomial::operator+=(const corecvs::Polynomial &rhs)
{
    size_t deg = rhs.degree();
    for (size_t i = 0; i <= deg; ++i)
        (*this)[i] += rhs[i];
    return *this;
}

corecvs::Polynomial  corecvs::Polynomial::operator- (const corecvs::Polynomial &rhs) const
{
    corecvs::Polynomial ans = *this;
    ans -= rhs;
    return ans;
}

corecvs::Polynomial& corecvs::Polynomial::operator-=(const corecvs::Polynomial &rhs)
{
    size_t deg = rhs.degree();
    for (size_t i = 0; i <= deg; ++i)
        (*this)[i] -= rhs[i];
    return *this;
}

std::ostream& operator<<(std::ostream& os, const corecvs::Polynomial &p)
{
    size_t deg = p.degree();
    os << std::showpos;
    for (size_t i = 0; i <= deg; ++i)
    {
        os << p[i] << "*x^" << i << " ";
    }
    return os;
}

corecvs::Polynomial::Polynomial(const double &x0)
{
    push_back(x0);
}

corecvs::Polynomial::Polynomial(const double *coeff, size_t degree)
    : std::vector<double>(coeff, coeff + degree + 1)
{
}

corecvs::Polynomial::Polynomial(const std::vector<double> &coeff)
    : std::vector<double>(coeff)
{
}

corecvs::Polynomial::Polynomial(const corecvs::Vector &coeff)
    : corecvs::Polynomial(&coeff.at(0), coeff.size() - 1)
{
}

corecvs::Polynomial corecvs::Polynomial::X()
{
    double c[] = {0.0, 1.0};
    return Polynomial(c, 1);
}

corecvs::Polynomial corecvs::Polynomial::FromRoots(const std::vector<double> &roots)
{
    corecvs::Polynomial ans = 1.0;
    for (auto& r: roots)
        ans *= corecvs::Polynomial::X() - r;
    return ans;
}