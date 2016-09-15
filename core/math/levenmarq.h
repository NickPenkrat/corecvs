#ifndef LEVEN_MARQ_H
#define LEVEN_MARQ_H

/**
 * \file levenmarq.h
 * \brief This class implements the Levenberg Marquardt algorithm
 *
 * \ingroup cppcorefiles
 * \date Mar 4, 2010
 * \author alexander
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits>
#include <vector>
#include <thread>

#include "global.h"

#include "vector2d.h"
#include "matrix.h"
#include "function.h"
#include "sparseMatrix.h"
#include "vector.h"
#include "minresQLP.h"
#include "pcg.h"
#include "statusTracker.h"
#include "preciseTimer.h"
#include "calculationStats.h"

namespace corecvs {

enum class LinearSolver
{
    NATIVE,
    SCHUR_COMPLEMENT,
    MINRESQLP,
    MINRESQLP_IC0,
    CG,
    PCG_IC0
};

template<typename MatrixClass, typename FunctionClass>
class LevenbergMarquardtImpl
{
private:
    enum LocalStats {
        FEVAL_TIME,
        JEVAL_TIME,
        LIN_SOLVE_TIME,
        JATA_TIME,
        TOTAL_TIME,
        LOCAL_STATS_LAST
    };

public:
    FunctionClass *f;
    //FunctionArgs *f;
    FunctionArgs *normalisation;
    double startLambda;
    double maxLambda;
    double lambdaFactor;
    int    maxIterations;

    bool traceProgress = true;
    bool traceCrucial  = false;
    bool trace         = false;
    bool traceMatrix   = false;
    bool traceJacobian = false;

    Statistics *stats;

    double f0, x0;
    double fTolerance = 1e-9,
           xTolerance = 1e-9;
#if 0
    bool useConjugatedGradient = false;
    int  conjugatedGradientIterations = 100;
#endif

    bool useExplicitInverse = false;
    bool terminateOnDegeneracy = false;
    LinearSolver linearSolver = LinearSolver::NATIVE;

    StatusTracker* state = nullptr;

    /* Additional outputs */
    bool hasParadox = false;
    int iterations = 0;

    /* The main ctor */
    LevenbergMarquardtImpl(int _maxIterations = 25, double _startLambda = 10, double _lambdaFactor = 2.0) :
        f(NULL),
        normalisation(NULL),
        startLambda(_startLambda),
        maxLambda(std::numeric_limits<double>::max()),
        lambdaFactor(_lambdaFactor),
        maxIterations(_maxIterations)
        {}

    /**
     *  Marquardt-Levenberg algoritm is a fast optimisation algorithm
     *
     *  \f[ (J^T J + \lambda diag(J^T J)) \delta = J^T [y_{target} - f(\beta)] \f]
     **/
    vector<double> fit(const vector<double> &input, const vector<double> &output)
    {
        if (traceProgress) {
            cout << "================== Starting LM fit (" << f->inputs << " -> " << f->outputs << ")================== " << endl;
            cout << "[" << std::flush;
        }

        Vector beta(input);
        Vector target(output);
        CORE_ASSERT_TRUE(f != NULL, "Function is NULL");
        CORE_ASSERT_TRUE_P(beta.size() == f->inputs, ("input guess has wrong dimension %d instead of %d\n", beta.size(), f->inputs));
        CORE_ASSERT_TRUE_P((int)output.size() == f->outputs, ("output has wrong dimension %d instead of %d\n", (int)output.size(), f->outputs));

        Vector y(f->outputs);    /**<Will hold the current function result*/
        Vector diff(f->outputs); /**<Will hold current difference to traget*/
        Vector delta(f->inputs);
        Vector yNew(f->outputs);
        Vector diffNew(f->outputs);

        FunctionArgs &F = *f;

        bool converged = false;

        double lambda = startLambda;
        double maxlambda = maxLambda;

        double norm = std::numeric_limits<double>::max();

        Vector initial = diff;
        F(beta, initial);
        initial -= target;
        double initialNorm = f0 = initial.sumAllElementsSq();
        x0 = !beta;
        int LSc = 0;

        //double totalEval = 0.0, totalJEval = 0.0, totalLinSolve = 0.0, totalATA = 0.0, totalTotal = 0.0;
        int64_t totalTimes[LOCAL_STATS_LAST] = {0};

        int g = 0;
        StatusTracker::Reset(state, "Fit", maxIterations);

        for (g = 0; (g < maxIterations) && (lambda < maxlambda) && !converged; g++)
        {
            int LSc_curr = 0;
            auto boo = StatusTracker::CreateAutoTrackerCalculationObject(state);

            int64_t iterationTimes[LOCAL_STATS_LAST] = {0};
            PreciseTimer start = PreciseTimer::CurrentETime();
            PreciseTimer interval;

            if (traceProgress && ((g % ((maxIterations / 100) + 1) == 0))) {
                cout << "#" << std::flush;
            }

            interval = PreciseTimer::CurrentETime();
            MatrixClass J = f->getNativeJacobian(&(beta[0]));
            iterationTimes[JEVAL_TIME] = interval.nsecsToNow();

            if (traceJacobian) {
                cout << "New Jacobian:" << endl << J << endl;
            }

            /*
             * Note: Using obscure profiling techniques we found that out L-M implementation is slow
             *       for big tasks. So we changed this stuff into calls to BLAS.
             *       For small tasks it may even decrease performance since calls to BLAS come
             *       with some non-zero cost.
             *       May be we need to investigate, from which problem size we should switch to BLAS.
             *
             * NOTE: Cool guys do not compute JTJ explicitly, since we can get all useful info from
             *       J's QR decomposition (Q term cancels out and is not needed explicitly),
             *       but we are using JTJ in user-enableable ouput, so I do not implement QR-way
             */
            interval = PreciseTimer::CurrentETime();
            MatrixClass JTJ = J.ata();
            iterationTimes[JATA_TIME] = interval.nsecsToNow();


            interval = PreciseTimer::CurrentETime();
            F(beta, y);
            iterationTimes[FEVAL_TIME] = interval.nsecsToNow();

            diff = target - y;
#if 0
                for (int ijk = 0; ijk < diff.size(); ++ijk)
                    CORE_ASSERT_TRUE_S(!std::isnan(diff[ijk]));
                for (int ijk = 0; ijk < y.size(); ++ijk)
                    CORE_ASSERT_TRUE_S(!std::isnan(y[ijk]));
                for (int ijk = 0; ijk < beta.size(); ++ijk)
                    CORE_ASSERT_TRUE_S(!std::isnan(beta[ijk]));
#endif
            Vector d = diff * J;
//                for (int ijk = 0; ijk < d.size(); ++ijk)
//                    CORE_ASSERT_TRUE_S(!std::isnan(d[ijk]));

            double normOld = norm;
            norm = diff.sumAllElementsSq();

            if (trace) {
                if (normOld < norm) {
                    cout << "Paradox: Norm has not decreased..." << (normOld - norm) << endl;
                    hasParadox = true;
                }

                cout << "Now  norm:" <<  norm << "( per param:" << sqrt(norm / diff.size()) << ") lambda:" << lambda << endl;
            }

            while (true)
            {
                if (norm == 0.0)
                {
                    if (traceCrucial)
                    {
                        cout << "Algorithm fully converged" << endl;
                    }
                    converged = true;
                    break;
                }

                if (!(lambda < std::numeric_limits<double>::max()))
                {
                    if (traceCrucial)
                    {
                        cout << "Algorithm seem to be trapped at point: " << endl;
                        cout << "After: " << g << " iterations" << endl;
                        cout << beta << endl;
                    }

                    if (traceMatrix)
                    {
                        cout << "Current Jacobian:" << endl << J << endl;
                        cout << "Current JTJ:" << endl << JTJ << endl;
                        cout << "previous delta was:" << endl << delta << endl;

                        int old = cout.precision(30);
                        cout << "guess:" << endl << beta << endl;
                        F(beta, yNew);
                        cout << "value:" << endl << yNew.l2Metric() << endl;
                        F(beta + delta, yNew);
                        cout << "value at +step:" << endl << yNew.l2Metric() << endl;
                        F(beta - delta, yNew);
                        cout << "value at -step:" << endl << yNew.l2Metric() << endl;
                        for (int i = 0; i < delta.size(); i++)
                        {
                            delta[i] = 0.0001 * J.a(0,i);
                        }
                        F(beta + delta, yNew);
                        cout << "value at +J:" << endl << yNew.l2Metric() << endl;
                        F(beta - delta, yNew);
                        cout << "value at -J:" << endl << yNew.l2Metric() << endl;
                        cout.precision(old);
                    }

                    break;
                }

                // Make a temporary copy
                MatrixClass A(JTJ);
                Vector B(d);

                for (int j = 0; j < A.h; j++)
                {
                    double a = A.a(j, j) + lambda;
                    //double b = A.a(j, j) * (1.0 + lambda);
                    A.a(j, j) = a;
                }

                /*
                 * NOTE: A'A (generally) is semi-positive-definite, but if you are
                 *       solving well-posed problems then diag(A'A) > 0 and A'A is
                 *       positive defined.
                 *       If you are experiencing strange problems with nans/etc,
                 *       try to make last flag "false" and examine your problem for
                 *       degeneracy
                 */

                LSc_curr++;
                LSc++;
                interval = PreciseTimer::CurrentETime();
                if (traceMatrix)
                    std::cout << A << std::endl << std::endl;
                bool shouldExit = false;
                switch (linearSolver)
                {
                    case LinearSolver::NATIVE:
                        A.linSolve(B, delta, true, true);
                        break;
                    case LinearSolver::SCHUR_COMPLEMENT:
                        CORE_ASSERT_TRUE_S(F.schurBlocks.size());
                        MatrixClass::LinSolveSchurComplement(A, B, F.schurBlocks, delta, true, true, useExplicitInverse);
                        break;
                    case LinearSolver::MINRESQLP:
                        {
                        auto res = MinresQLP<MatrixClass>::Solve(A, B, delta);
                        auto P = A.incompleteCholseky();
                        auto PP = [&](const Vector& x)->Vector { return P.second.dtrsv_un(P.second.dtrsv_ut(x)); };
                        auto res2 = MinresQLP<MatrixClass>::Solve(A, PP, B, delta);
                        if (res != res2)
                        {
                            std::cout << "CURIOUS: NON-PRECONDITIONED:" << res << " PRECONDITIONED: " << res2 << std::endl;
                        }
                        if (res != MinresQLPStatus::SOLVED_RTOL &&
                            res != MinresQLPStatus::SOLVED_EPS  &&
                            res != MinresQLPStatus::MINLEN_RTOL &&
                            res != MinresQLPStatus::MINLEN_EPS  &&
                            terminateOnDegeneracy)
                            shouldExit = true;
                        }
                        break;
                    case LinearSolver::MINRESQLP_IC0:
                        {
#ifndef WITH_CUSPARSE
                        auto P123 = A.incompleteCholseky();
#else
                        decltype(A.incompleteCholseky()) P123;
                        std::thread t1([&](){ P123 = A.incompleteCholseky(); std::cout << "IC0 completed" << std::endl;});
                        std::thread t2([&]() { A.promoteToGpu(); for (int i = 0; i < 3; ++i) auto vv = A * B; });
                        t1.join();
                        t2.join();
#endif
                        MinresQLPStatus res123;
                        if (P123.first)
                        {
                            auto PP123 = [&](const Vector& x)->Vector { return P123.second.dtrsv_un(P123.second.dtrsv_ut(x)); };
                            res123 = MinresQLP<MatrixClass>::Solve(A, PP123, B, delta);
                        }
                        else
                        {
                            std::cout << "!!!IC0 failed, running without preconditioner!!!" << std::endl;
                            res123 = MinresQLP<MatrixClass>::Solve(A, B, delta);
                        }
                        if (res123 != MinresQLPStatus::SOLVED_RTOL &&
                            res123 != MinresQLPStatus::SOLVED_EPS  &&
                            res123 != MinresQLPStatus::MINLEN_RTOL &&
                            res123 != MinresQLPStatus::MINLEN_EPS  &&
                            terminateOnDegeneracy)
                            shouldExit = true;
                        }
                        break;
                    case LinearSolver::CG:
                        PCG<MatrixClass>::Solve(A, B, delta);
                        break;
                    case LinearSolver::PCG_IC0:
                        auto P = A.incompleteCholseky();
                        auto PP = [&](const Vector& x)->Vector { return P.second.dtrsv_un(P.second.dtrsv_ut(x)); };
                        auto res2 = PCG<MatrixClass>::Solve(A, PP, B, delta);
                        if (res2 != PCGStatus::CONVERGED && terminateOnDegeneracy)
                            shouldExit = true;
                        break;
                }
                if (shouldExit)
                {
                    converged = true;
                    break;
                }
                iterationTimes[LIN_SOLVE_TIME] = interval.nsecsToNow();

//                for (int ijk = 0; ijk < delta.size(); ++ijk)
//                    CORE_ASSERT_TRUE_S(!std::isnan(delta[ijk]));

                interval = PreciseTimer::CurrentETime();
                F(beta + delta, yNew);
                iterationTimes[FEVAL_TIME] += interval.nsecsToNow();

                diffNew = target - yNew;
                double normNew = diffNew.sumAllElementsSq();
                if (trace) {
                    cout << "  Guess:" <<  normNew << " - ";
                }

                if (normNew < norm) // If the current solution is better
                {
                    if (trace) {
                        cout << "Accepted" << endl;
                        std::cout << initialNorm - normNew << " decrease in " << LSc << " linsolves" << std::endl;
                        std::cout << norm - normNew << " current decrease (" << LSc_curr << " linsolves" << std::endl;
                        std::cout << "NLO: " << initialNorm - normNew << ", " << LSc << ", " << norm - normNew << ", " << LSc_curr << ", " << lambda << std::endl;
                    }

                    if (traceMatrix) {
                        cout << "Old soluton:" << endl << beta << endl;
                    }
                    double funTolerance = (norm - normNew) / f0;// / (1.0 + norm);
                    double xxTolerance = !delta / x0;// / (1.0 + !beta);
                    if (trace)
                    std::cout << "Tolerances: |f(x)-f(x+d)|/(1.0+f(x): " << funTolerance << " |d|/|1+|x||: " << xxTolerance << std::endl;
                    if (funTolerance < fTolerance && xxTolerance < xTolerance)
                    {
                        converged = true;
                        if (trace)
                        std::cout << "CONVERGED (FTOL+XTOL)" << std::endl;
                    }
                    lambda /= lambdaFactor;
                    norm = normNew;
                    beta += delta;
                    if (normalisation != NULL)
                    {
                        Vector normBeta(beta.size());
                        normalisation->operator()(beta, normBeta);
                        beta = normBeta;
                    }

                    if (traceMatrix) {
                        cout << "New soluton:" << endl << beta << " - " << normNew << endl;
                    }
                    break;
                }
                else
                {
                    if (trace) {
                        cout << "Rejected lambda old: "<< lambda << " lambda new:" << lambda * lambdaFactor << endl;
                    }
                    lambda *= lambdaFactor; // Current solution is worse. Try new lambda
                    if (traceMatrix) {
                        cout << "Keep soluton:" << endl << beta << " - " << normNew << " l:"<<  lambda << endl;
                    }
                }
            }

            iterationTimes[TOTAL_TIME] += start.nsecsToNow();
    #if 1
            if (traceProgress)
            {
                printStats(iterationTimes);
            }
    #endif
            for (int statId = 0; statId < LOCAL_STATS_LAST; statId++)
            {
                totalTimes[statId] += iterationTimes[statId];
            }
        }

        if (traceProgress) {
            printStats(totalTimes);
        }

        if (traceProgress) {
            cout << "]" << endl;
        }

        vector<double> result;
        result.reserve(f->inputs);
        for (int i = 0; i < f->inputs; i++) {
            result.push_back(beta[i]);
        }

        iterations = g;
        return result;
    }


    void printStats(int64_t stats[LOCAL_STATS_LAST])
    {
        double secStats[LOCAL_STATS_LAST];
        for (int statId = 0; statId < LOCAL_STATS_LAST; statId++)
        {
            secStats[statId] = stats[statId] / 1e9;
        }
        double leftover = secStats[TOTAL_TIME] -  secStats[FEVAL_TIME] - secStats[JEVAL_TIME] - secStats[JATA_TIME] - secStats[LIN_SOLVE_TIME];

        std::cout << "Total : " << secStats[TOTAL_TIME]     << "s " << std::endl
                  << "Eval  : " << secStats[FEVAL_TIME]     << "s (" << secStats[FEVAL_TIME]     / secStats[TOTAL_TIME] * 100.0 << ")" << std::endl
                  << "JEval : " << secStats[JEVAL_TIME]     << "s (" << secStats[JEVAL_TIME]     / secStats[TOTAL_TIME] * 100.0 << ")" << std::endl
                  << "ATA   : " << secStats[JATA_TIME]      << "s (" << secStats[JATA_TIME]      / secStats[TOTAL_TIME] * 100.0 << ")" << std::endl
                  << "LS    : " << secStats[LIN_SOLVE_TIME] << "s (" << secStats[LIN_SOLVE_TIME] / secStats[TOTAL_TIME] * 100.0 << ")" << std::endl
                  << "Other : " << leftover << "s (" << leftover / secStats[TOTAL_TIME] * 100.0 << ")" << std::endl;
    }


#if 0
    corecvs::Vector conjugatedGradient(const MatrixClass &A, const corecvs::Vector &B)
    {
        corecvs::Vector X(A.w), R = B, p = B;
        double rho0 = B.sumAllElementsSq(), rho1 = 0.0;
        for (int i = 0; i < A.w; ++i) X[i] = 0.0;

        static int cgf = 0, cgo = 0;

        std::cout << "PRECG: " << !(A*X-B) << std::endl;
        double eps = 1e-9 * rho0;
        for (int i = 0; i < conjugatedGradientIterations; ++i)
        {
            p = i ? R + (rho0 / rho1) * p : R;
            auto w = A * p;
            double alpha = rho0 / (w & p);
            X = X + alpha * p;
            R = A * X - B;
            rho1 = rho0;
            rho0 = R.sumAllElementsSq();
            // XXX: sometimes matrices are close to degenerate, so
            //      CG-step may fail
            if (rho0 < eps)
            {
                break;
            }
            if (rho1 < rho0)
                cgf++;
            else
                cgo++;
        }
        if ((A*X-B).sumAllElementsSq() > B.sumAllElementsSq())
        {
            corecvs::Vector v;
            A.linSolve(B, v);
            return v;
        }
        std::cout << "POST-CG: " << !(A*X-B) << " cg failures: " << ((double)cgf) / (double)((cgf + cgo + 1.0)) * 100.0 << "%" << std::endl;
        return X;
    }
#endif
};

typedef LevenbergMarquardtImpl<Matrix, FunctionArgs> LevenbergMarquardt;
typedef LevenbergMarquardtImpl<SparseMatrix, SparseFunctionArgs> LevenbergMarquardtSparse;

} //namespace corecvs
#endif

