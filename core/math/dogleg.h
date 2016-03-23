#ifndef DOGLEG_H
#define DOGLEG_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <limits>
#include <vector>
#include <chrono>

#include "global.h"

#include "vector2d.h"
#include "matrix.h"
#include "function.h"
#include "sparseMatrix.h"

#include "vector.h"
#include "sparseMatrix.h"


namespace corecvs {

template<typename MatrixClass, typename FunctionClass>
class DogLegImpl
{
public:
    FunctionClass *f;
    FunctionArgs *normalisation;
    double maxLambda;
    double lambdaFactor;
    int    maxIterations;

    bool traceProgress = true;
    bool traceCrucial  = false;
    bool trace         = false;

    int iterations = 0;

    Vector x, r, g, y, v, p, epsilon_p,
           hsd, hgn, hdl;
    bool gnReady = false;
    MatrixClass J, JTJ;
    double trustRegion, rho,
           diff_old;



    /* The main ctor */
    DogLegImpl(int _maxIterations = 25, double _startLambda = 10, double _lambdaFactor = 2.0) :
        f(NULL),
        normalisation(NULL),
        maxIterations(_maxIterations),
        trustRegion(1.0)
        {}

    vector<double> fit(const vector<double> &input, const vector<double> &output)
    {
        if (traceProgress)
        {
            cout << "================== Starting DL fit ================== " << endl;
            cout << "[" << std::flush;
        }

        CORE_ASSERT_TRUE(f != NULL, "Function is NULL");
        CORE_ASSERT_TRUE_P((int)output.size() == f->outputs, ("output has wrong dimension %d instead of %d\n", (int)output.size(), f->outputs));

        p = Vector(input);
        x = Vector(output);
        v = Vector(output);
        hdl = Vector(input);
        hsd = Vector(input);


        (*f)(p, v);

        epsilon_p = (x - v);
        J = f->getNativeJacobian(&p[0]);
        JTJ = J.ata();
        g = epsilon_p * J;

        int k;
        for (k = 0; k < maxIterations && std::numeric_limits<double>::min() < trustRegion; ++k)
        {
            if (k % ((maxIterations + 99) / 100) == 0)
                std::cout << "#" << std::flush;

            hsd = (g & g) / ((g * JTJ) & g) * hsd;
            gnReady = false;
            do
            {
                if (!hsd >= trustRegion)
                {
                    hdl = trustRegion / (!hsd) * hsd;
                    std::cout << "JSD" << std::endl;
                }
                else
                {
                    if (!gnReady)
                    {
                        hgn = JTJ.linSolve(g, true);
                        gnReady = true;
                    }
                    if (!hgn < trustRegion)
                    {
                        std::cout << "HGN" << std::endl;
                        hdl = hgn;
                    }
                    else
                    {
                        std::cout << "HGN&JSD" << std::endl;
                        auto A = hsd - hgn;
                        auto B = hgn;
                        double a = A & A,
                            b = 2.0 * A & B,
                            c = (B & B) - trustRegion * trustRegion,
                            d = std::max(b * b - 4 * a * c, 0.0);
                        double beta = (-b + std::sqrt(d)) / 2.0 / a;
                        hdl = beta * A + B;
//                        CORE_ASSERT_TRUE_S(std::abs(trustRegion - !hdl) < 1e-3);
                    }
                }

                auto pn = p + hdl;
                (*f)(pn, v);

                auto diff = x - v;
                double diff_new = diff & diff;
                rho = ((epsilon_p & epsilon_p) - (diff & diff)) / (2.0 * (epsilon_p & (J * hdl)) - (hdl & (JTJ * hdl)));
                std::cout << trustRegion << "TR" << std::endl;
                if (rho > 0.0)
                {
                    p = pn;
                    J = f->getNativeJacobian(&p[0]);
                    JTJ = J.ata();
                    (*f)(p, v);
                    epsilon_p = x - v;
                    g = epsilon_p * J;
                    std::cout << x << std::endl;
                    std::cout << "RES:" <<  !epsilon_p << std::endl;
                } else
                {
                    std::cout << "FAIL!" << trustRegion << "/" << rho << std::endl;
                }

                if (std::isnan(rho))
                    break;
                trustRegion = rho > 0.75 ? std::max(trustRegion, 3.0 * !hdl) : rho < 0.25 ? trustRegion / 2.0 : trustRegion;
            } while(rho <= 0.0);
            if (std::isnan(rho))
                break;
        }
        std::cout << "<<RET>>> " << k << std::endl;
        return std::vector<double>(&p[0], &p[f->inputs]);
    }
};

typedef DogLegImpl<Matrix, FunctionArgs> DogLeg;
typedef DogLegImpl<SparseMatrix, SparseFunctionArgs> DogLegSparse;
}

#endif