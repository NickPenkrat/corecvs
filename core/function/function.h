#ifndef _FUNCTION_H_
#define _FUNCTION_H_
/**
 * \file function.h
 * \brief a header that contains generic function
 *
 *
 * \ingroup cppcorefiles
 * \date Oct 28, 2009
 * \author alexander
 */
#include <vector>
#include <chrono>

#include "global.h"

#include "cblasLapackeWrapper.h"

#include "matrix.h"
#include "sparseMatrix.h"
#include "vector.h"

namespace corecvs {
using std::vector;

/**
 *  This class is a virtual version of a \f$ f: R^n \mapsto R^m\f$
 *
 *  TODO: Think of the similar class based on static polymorphism.
 **/
class FunctionArgs
{
public:
    int inputs;
    int outputs;

    FunctionArgs(int _inputs, int _outputs) :
        inputs (_inputs),
        outputs (_outputs)
    {}


    /**
     *  Operator that computes funсtion
     **/
    virtual void operator()(const double in[], double out[]) = 0;

    virtual void operator()(const Vector &in, Vector &out)
    {
        return operator()(in.element, out.element);
    }

    virtual void operator()(const vector<double> &in, vector<double> &out)
    {
        CORE_ASSERT_TRUE( (int)  in.size() > inputs , "Too few input numbers");
        CORE_ASSERT_TRUE( (int) out.size() > outputs, "Too few output numbers");

        return operator()(&in[0], &out[1]);
    }


    /**
     *  This function computes Jacobian
     *   \f[
     *
     *   J= \pmatrix{
     *      \frac{\partial y_1}{\partial x_1} & \cdots & \frac{\partial y_1}{\partial x_n} \cr
     *              \vdots                    & \ddots &                   \vdots          \cr
     *      \frac{\partial y_m}{\partial x_1} & \cdots & \frac{\partial y_m}{\partial x_n}
     *      }
     *
     *   \f]
     *
     *
     *   TODO: Bring differencing out and use following information
     *   http://en.wikipedia.org/wiki/Numerical_differentiation#Practical_considerations_using_floating_point_arithmetic
     **/
    virtual Matrix getJacobian(const double in[], double delta = 1e-7)
    {
        Matrix result(outputs, inputs);
        vector<double> xc(inputs);
        vector<double> y_minus(outputs);
        vector<double> y_plus (outputs);

        for (int i = 0; i < inputs; i++)
        {
            xc[i] = in[i];
        }

        for (int i = 0; i < inputs; i++)
        {
            double delta_loc = std::max(1.0, std::abs(in[i])) * delta;
            double xm = xc[i] = in[i] - delta_loc;
            operator()(&xc[0], &y_minus[0]);
            double xp = xc[i] = in[i] + delta_loc;
            operator()(&xc[0], &y_plus[0]);
            xc[i] = in[i];

            // Note: this stuff is not equal to 2 * delta
            double dx = xp - xm;
            for (int j = 0; j < outputs; j++)
            {
                result.element(j,i) = (y_plus[j] - y_minus[j]) / dx;
            }
        }
        return result;
    }

    Matrix getNativeJacobian(const double in[], double delta = 1e-7)
    {
        return getJacobian(in, delta);
    }

    virtual Matrix getJacobian(const Vector &in, double delta = 1e-7)
    {
        return getJacobian(in.element, delta);
    }

    virtual ~FunctionArgs() {}

};

class SparseFunctionArgs : public FunctionArgs
{
public:
    double feval = 0.0, transp = 0.0, construct = 0.0, prepare = 0.0, other = 0.0, subscale = 0.0;
    SparseFunctionArgs(int inputs, int outputs, const std::vector<std::vector<int>> &dependencyList) : FunctionArgs(inputs, outputs), dependencyList(dependencyList), fullIdx(outputs)
    {
        std::cout << "Sparse: R^" << inputs << "->R^" << outputs << std::endl;
        for (int i = 0; i < outputs; ++i)
            fullIdx[i] = i;
        minify();
    }
    //! \brief This should compute only needed indices
    virtual void operator() (const double* in, double* out, const std::vector<int> &idx) = 0;
    virtual void operator() (const double* in, double* out)
    {
        (*this)(in, out, fullIdx);
    }

    Matrix getJacobian(const double* in, double delta = 1e-7)
    {
        return (Matrix)getNativeJacobian(in, delta);
    }

    SparseMatrix getNativeJacobian(const double* in, double delta = 1e-7)
    {
        std::vector<std::vector<double>> values(inputs);
        for (int i = 0; i < inputs; ++i)
            values[i].reserve(dependencyList[i].size());

        int N = (int)groupInputs.size();
        std::vector<double> xp(inputs), xm(inputs), deltaS(inputs);
        double curr_eval = 0.0, curr_subscale = 0.0;
        auto start_evals = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < N; ++i)
        {
            auto& group = groupInputs[i];
            auto& idxs = groupOutputs[i];
            auto& remap = remapIdx[i];

            int M = (int)group.size();

            for (int j = 0; j < inputs; ++j)
                xp[j] = xm[j] = in[j];
            for (int j = 0; j < M; ++j)
            {
                double x = in[group[j]];
                double delta_loc = std::max(1.0, std::abs(x)) * delta;
                double xxm = x - delta_loc;
                double xxp = x + delta_loc;
                xp[group[j]] = xxp;
                xm[group[j]] = xxm;
                deltaS[j] = xxp - xxm;
            }

            std::vector<double> yp(idxs.size()), ym(idxs.size());
            auto begin_eval = std::chrono::high_resolution_clock::now();
            operator()(&xp[0], &yp[0], idxs);
            operator()(&xm[0], &ym[0], idxs);
            auto end_eval = std::chrono::high_resolution_clock::now();
            curr_eval += (end_eval - begin_eval).count() / 1e9;

            auto begin_subscale = std::chrono::high_resolution_clock::now();
            int K = (int)idxs.size();
#ifdef WITH_BLAS
            cblas_daxpy(K, -1.0, &ym[0], 1, &yp[0], 1);
#endif
            for (int j = 0; j < K; ++j)
            {
                int id = remap[idxs[j]];
#ifdef WITH_BLAS
                double v = yp[j] / deltaS[id];
#else
                double v = (yp[j] - ym[j]) / deltaS[id];
#endif
                values[group[id]].push_back(v);
            }
            auto end_subscale = std::chrono::high_resolution_clock::now();
            curr_subscale += (end_subscale - begin_subscale).count() / 1e9;
        }
        auto end_evals = std::chrono::high_resolution_clock::now();
        double eval_f = curr_eval;
        double eval_other = (end_evals - start_evals).count() / 1e9 - eval_f - curr_subscale;


        auto begin_prepare = std::chrono::high_resolution_clock::now();
        std::vector<double> sparseValues;
        std::vector<int> sparseColumns, sparseRowPointers(inputs + 1);
        for (int i = 0; i < inputs; ++i)
        {
            int N = (int)dependencyList[i].size();
            CORE_ASSERT_TRUE_S(N == (int)values[i].size());
            for (int j = 0; j < N; ++j)
            {
                int jj = dependencyList[i][j];
                sparseColumns.push_back(jj);
                sparseValues.push_back(values[i][j]);
            }
            sparseRowPointers[i + 1] = (int)sparseValues.size();
        }
        auto end_prepare = std::chrono::high_resolution_clock::now();
        auto begin_construct = std::chrono::high_resolution_clock::now();
        auto sm = SparseMatrix(inputs, outputs, std::move(sparseValues), std::move(sparseColumns), std::move(sparseRowPointers));
        auto end_construct = std::chrono::high_resolution_clock::now();
        auto begin_t = std::chrono::high_resolution_clock::now();
        auto sm_t = sm.t();
        auto end_t = std::chrono::high_resolution_clock::now();

        double prepare_t = (end_prepare - begin_prepare).count() / 1e9;
        double construct_t = (end_construct - begin_construct).count() / 1e9;
        double t_t = (end_t - begin_t).count() / 1e9;
        feval += eval_f;
        transp += t_t;
        construct += construct_t;
        prepare += prepare_t;
        other += eval_other;
        subscale += curr_subscale;
        return sm_t;
    }

    void minify()
    {
        std::vector<int> usedO(outputs);
        std::vector<int> usedI(inputs);
        CORE_ASSERT_TRUE_S((int)dependencyList.size() == inputs);

        for (int i = 0; i < inputs; ++i)
        {
            if (usedI[i])
                continue;
            std::vector<int> currentGroup = {i}, currentRemap(outputs, -1), currentOutputs = {};
            usedO.clear();
            usedO.resize(outputs);
            for (auto& id: dependencyList[i])
            {
                CORE_ASSERT_TRUE_S(id < outputs);
                usedO[id] = 1;
            }
            for (int j = i + 1; j < inputs; ++j)
            {
                if (usedI[j])
                    continue;
                bool isOk = true;
                for (auto& id: dependencyList[j])
                    if (usedO[id])
                    {
                        isOk = false;
                        break;
                    }
                if (!isOk)
                    continue;
                currentGroup.push_back(j);
                usedI[j] = 1;
                for (auto& id: dependencyList[j])
                    usedO[id] = 1;
            }
            CORE_ASSERT_TRUE_S((int)currentRemap.size() == outputs);
            for (auto& id: currentGroup)
                for (auto& ido: dependencyList[id])
                {
                    currentOutputs.push_back(ido);
                    currentRemap[ido] = &id - &*currentGroup.begin();
                }
            groupInputs.push_back(currentGroup);
            groupOutputs.push_back(currentOutputs);
            remapIdx.push_back(currentRemap);

        }
        std::cout << "REMAPANAL: " << inputs << "->" << groupInputs.size() << std::endl;
    }

    virtual ~SparseFunctionArgs()
    {
        double total = (feval + transp + construct + prepare + other + subscale);
        std::cout << "Feval: " << feval << "s " << feval / total * 100.0 << "%" << std::endl;
        std::cout << "Trans: " << transp<< "s " << transp/ total * 100.0 << "%" << std::endl;
        std::cout << "Prep : " <<prepare << "s " <<prepare/ total * 100.0 << "%" << std::endl;
        std::cout << "Const: " <<construct<< "s " <<construct / total * 100.0 << "%" << std::endl;
        std::cout << "Subscale:" << subscale << "s " << subscale / total * 100.0 << "%" << std::endl;
        std::cout << "Other: " <<other<< "s " <<other / total * 100.0 << "%" << std::endl;
    }

private:
    std::vector<std::vector<int>> groupInputs, groupOutputs, remapIdx;
    std::vector<std::vector<int>> dependencyList;
    std::vector<int>              fullIdx;
};

class IdentityFunction : public FunctionArgs
{
public:
    IdentityFunction(int dimension) : FunctionArgs(dimension, dimension) {}

    virtual void operator()(const double in[], double out[])
    {
        for (int i = 0; i < inputs; i++)
            out[i] = in[i];
    }

    virtual Matrix getJacobian(const double /*in*/[], double /*delta*/ = 1e-7)
    {
        Matrix result(outputs, inputs, 1.0);
        return result;
    }
};


class ScalerFunction : public FunctionArgs
{
public:
    FunctionArgs *F;
    double scale;
    double shift;


    ScalerFunction(FunctionArgs *_F, double _scale, double _shift) :
        FunctionArgs(_F->inputs, _F->outputs),
        F(_F),
        scale(_scale),
        shift(_shift)
    {
        cout << inputs << " " << outputs << endl;
        cout << F->inputs << " " << F->outputs << endl;
    }

    virtual void operator()(const double in[], double out[])
    {
        F->operator ()(in,out);
        for (int i = 0; i < outputs; i++)
            out[i] = out[i] * scale + shift;
    }
};

template <class Map2D>
class Map2DFunction : public FunctionArgs
{
    Map2D *mMap;

public:
    Map2DFunction(Map2D *map) :
        FunctionArgs(2, 2),
        mMap(map)
    {}

    virtual void operator()(const double in[], double out[])
    {
        Vector2dd mapResult = mMap->map(in[0], in[1]);
        out[1] = mapResult.x() - in[1];
        out[0] = mapResult.y() - in[0];
    }
};


class LengthFunction : public FunctionArgs
{
    FunctionArgs *F;

public:
    LengthFunction(FunctionArgs *_F) :
        FunctionArgs(_F->inputs, 1),
        F(_F)
    {
        cout << inputs << " " << outputs << endl;
        cout << F->inputs << " " << F->outputs << endl;
    }

    virtual void operator()(const double in[], double out[])
    {
        vector<double> tmpOut(F->outputs);

        F->operator ()(in, &tmpOut[0]);
        out[0] = 0.0;
        for (int i = 0; i < F->outputs; i++) {
            out[0] += tmpOut[i] * tmpOut[i];
        }
        out[0] = sqrt(out[0]);
    }
};


template<typename RealFuncType, int inputDim, int outputDim>
class FunctionStatic
{
public:
    typedef FixedVector<double, inputDim>   InputType;
    typedef FixedVector<double, outputDim> OutputType;

    void operator()(const InputType &/*in*/, OutputType &/*out*/) {}

    OutputType f(const InputType &in)
    {
        OutputType result;
        RealFuncType *realThis = (RealFuncType *)this;
        realThis->operator()(in, result);
        return result;
    }

    virtual OutputType vf(const InputType &in)
    {
        OutputType result;
        RealFuncType *realThis = (RealFuncType *)this;
        realThis->operator()(in, result);
        return result;
    }

#if 0
    /**
     *   \f[
     *
     *   J= \pmatrix{
     *      \frac{\partial y_1}{\partial x_1} & \cdots & \frac{\partial y_1}{\partial x_n} \cr
     *              \vdots                    & \ddots &                   \vdots          \cr
     *      \frac{\partial y_m}{\partial x_1} & \cdots & \frac{\partial y_m}{\partial x_n}
     *      }
     *
     *   \f]
     *
     *
     *   TODO: Bring differencing out and use following information
     *   http://en.wikipedia.org/wiki/Numerical_differentiation#Practical_considerations_using_floating_point_arithmetic
     **/
    Matrix getJacobian(const InputType &x, double delta = 1e-7)
    {
        Matrix result(outputDim, inputDim);
        for (int i = 0; i < inputDim; i++)
        {
           Function::InputType xc = x;
           xc[i] -= delta;
           Function::InputType y_minus;
           Function::InputType y_plus;
           F(xc, y_minus);
           xc[i] = x[i] + delta;
           F(xc, y_plus);

           for (int j = 0; j < Function::outputDim; j++)
           {
               result.element(j,i) = (y_plus[j] - y_minus[j]) / (2.0 * delta);
           }
        }
        return result;
    }
#endif
};

#if 0
double const delta = 0.1;

class Function
{
public:
    Function()
    {
        memset(args, '\0', sizeof(float) * dimension);
    }

    static const int dimension = 3;

    float operator ()(float a, float b, float c)
    {
        return a + b*b + c*c*c;
    }

    float operator()()
    {
        return args[0] + args[1]*args[1] + args[2] * args[2] * args[2];
    }

    void setArg(int argNum, double arg)
    {
        args[argNum] = arg;
    }

    void modifyArg(int argNum, double delta)
    {
        args[argNum] += delta;
    }

private:
    double args[dimension];

};

template <typename T>
class Derivative
{
public:
    float operator ()(int number, ...)
    {
        T f;
        number--;
        double val = 0.0;
        va_list vl;
        va_start(vl, number);
        for (int i=0; i < Function::dimension; i++)
        {
            val = va_arg(vl, double);
            f.setArg(i, val);
        }
        va_end(vl);
        f.modifyArg(number, -delta);
        double f1 = f();
        f.modifyArg(number, 2 * delta);
        double f2 = f();
        return (f2 - f1) / (2 * delta);
    }
};
#endif

} //namespace corecvs

#endif // _FUNCTION_H_
