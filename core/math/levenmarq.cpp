#include <stdio.h>
#include <string.h>
#include <math.h>
#include <limits>

#include "global.h"

#include "levenmarq.h"
#include "stdlib.h"
#include "vector.h"
#include "sparseMatrix.h"

namespace corecvs {

//#define TRACE_PROGRESS
//#define TRACE_CRUCIAL
//#define TRACE
//#define TRACE_MATRIX


using std::flush;
/**
 *  Marquardt-Levenberg algoritim is a fast optimisation algorithm
 *
 *  \f[ (J^T J + \lambda diag(J^T J)) \delta = J^T [y_{target} - f(\beta)] \f]
 **/
vector<double> LevenbergMarquardt::fit(const vector<double> &input, const vector<double> &output)
{
    if (traceProgress) {
        cout << "================== Starting LM fit ================== " << endl;
        cout << "[" << flush;
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

    for (int g = 0; (g < maxIterations) && (lambda < maxlambda) && !converged; g++)
    {
        if (traceProgress) {
            if ((g % ((maxIterations / 100) + 1) == 0))
            {
                cout << "#" << flush;
            }
        }

        Matrix J = f->getJacobian(&(beta[0]));

        if (traceJacobian) {
            cout << "New Jacobian:" << endl << J << endl;
        }

        Matrix JTJ = J.ata();

        F(beta, y);
        diff = target - y;
        Vector d = diff * J;

        double normOld = norm;
        norm = diff.sumAllElementsSq();

        if (trace) {
            if (normOld < norm) {
                cout << "Paradox: Norm has not decreased..." << (normOld - norm) << endl;
                hasParadox = true;
            }

            cout << "Now  :" <<  norm << " " << lambda << endl;
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
            Matrix A(JTJ);
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
            delta = A.linSolve(B, true, true);
            F(beta + delta, yNew);
            diffNew = target - yNew;
            double normNew = diffNew.sumAllElementsSq();
            if (trace) {
                cout << "  Guess:" <<  normNew << " - ";
            }

            if (normNew < norm) // If the current solution is better
            {
                if (trace) {
                    cout << "Accepted" << endl;
                }

                if (traceMatrix) {
                    cout << "Old soluton:" << endl << beta << endl;
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
    }

    if (traceProgress) {
        cout << "]" << endl;
    }

    vector<double> result;
    result.reserve(f->inputs);
    for (int i = 0; i < f->inputs; i++) {
        result.push_back(beta[i]);
    }
    return result;
}

} //namespace corecvs

