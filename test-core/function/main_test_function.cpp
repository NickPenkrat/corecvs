#include "function.h"
#include "gtest/gtest.h"
#include "global.h"

struct CountingFunctor
{
    double operator() (const double* in, int i)
    {
        counter++;
        double res = 0.0;
        for (int j = 0; j < i; ++j)
            res += in[j];
        return res;
    }
    static int counter;
};

int CountingFunctor::counter = 0;

struct SparseCountingFunctor : corecvs::SparseFunctionArgs
{
    SparseCountingFunctor(int inputs, int outputs) : SparseFunctionArgs(inputs, outputs, generateSparsityList(inputs, outputs))
    {
    }
    void operator() (const double *in, double *out, const std::vector<int> &idx)
    {
        for (size_t i = 0; i < idx.size(); ++i)
            out[i] = cf(in, idx[i]);
    }
    static std::vector<std::vector<int>> generateSparsityList(int inputs, int outputs)
    {
        std::vector<std::vector<int>> sparsity;
        sparsity.resize(inputs);
        for (int i = 0; i < inputs; ++i)
            for (int j = 0; j < outputs; ++j)
                if (j > i)
                    sparsity[i].push_back(j);
        return sparsity;
    }
    CountingFunctor cf;
};

struct NonSparseCountingFunctor : corecvs::FunctionArgs
{
    NonSparseCountingFunctor(int inputs, int outputs) : FunctionArgs(inputs, outputs)
    {
    }
    void operator() (const double *in, double *out)
    {
        for (int i = 0; i < outputs; ++i)
            out[i] = cf(in, i);
    }
    CountingFunctor cf;
};

TEST(SparseFunction, JacobianComputation)
{
    SparseCountingFunctor scf(100, 100);
    std::vector<double> pt(100);
    for (int i = 0; i < 100; ++i)
        pt[i] = i;
    auto J = scf.getJacobian(&pt[0]);
    int cntSparse = CountingFunctor::counter;
    CountingFunctor::counter = 0;
    NonSparseCountingFunctor nscf(100, 100);
    auto J2=nscf.getJacobian(&pt[0]);
    int cntNonSparse = CountingFunctor::counter;
    ASSERT_TRUE(cntNonSparse > 2 * cntSparse);
    ASSERT_LE((J - J2).frobeniusNorm(), 1e-9);
}
