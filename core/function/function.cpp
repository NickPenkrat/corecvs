#include "function.h"

namespace corecvs
{
Matrix FunctionArgs::getLSQHessian(const double* in)
{
    // H_ij = \sum f^k_ij*f^k + f^ki f^kj
    JacobianFunctor jf(this);
    Vector v(outputs);
    (*this)(in, &v[0]);
    Matrix jj = jf.getJacobian(in, 1e-9);
    Matrix J = getJacobian(in, 1e-9);
    Matrix H(inputs, inputs);
    for (int i = 0; i < inputs; ++i)
    {
        for (int j = 0; j < inputs; ++j)
        {
            H.a(i, j) = 0.0;
            for (int k = 0; k < outputs; ++k)
            {
                H.a(i, j) += J.a(k, i) * J.a(k, j) + v[k] * jj.a(j, k * inputs + i);
            }
        }
    }
    return H;
}
};
