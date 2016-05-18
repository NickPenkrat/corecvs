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
    bool useSchurComplement = false;

    int iterations = 0;

    Vector x, r, g, y, v, p, epsilon_p,
           hsd, hgn, hdl;
    double augmentation = 0.0,
           augmentMin = 1e-9,
           augmentStep= 2;
    int    linsolveOk = 0,
           dumpingDecrease = 3;
    bool gnReady = false;
    MatrixClass J, JTJ;
    double trustRegion, rho,
           diff_old;

    DogLegImpl(int _maxIterations = 25) :
        f(NULL),
        normalisation(NULL),
        maxIterations(_maxIterations),
        trustRegion(1.0)
        {}

    void linSolveDumped(MatrixClass &A, Vector &B, Vector &res)
    {
        if (augmentation > 0.0 && linsolveOk > dumpingDecrease)
        {
            augmentation /= augmentStep;
            if (augmentation < augmentMin)
                augmentation = 0.0;
            linsolveOk = 0;
        }
        do
        {
            MatrixClass AA(A);
            if (augmentation > 0.0)
            {
                double maxAbs = 0.0;
                for (int i = 0; i < AA.w; ++i)
                    maxAbs = std::max(maxAbs, std::abs(AA.a(i, i)));
                if (maxAbs == 0.0)
                    maxAbs = 1.0;
                for (int i = 0; i < AA.w; ++i)
                    AA.a(i, i) += maxAbs * augmentation;
            }
            bool ok = !useSchurComplement ? AA.linSolve(B, res, true, true) : AA.linSolveSchurComplement(B, f->schurBlocks, res, true, true);
            if (ok)
            {
                linsolveOk++;
                break;
            }
            augmentation = augmentation == 0.0 ? augmentMin : augmentation * augmentStep;
            linsolveOk = 0;
        } while (1);
    }

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
                if ((!hsd) >= trustRegion)
                {
                    hdl = trustRegion / (!hsd) * hsd;
                    std::cout << "JSD" << std::endl;
                }
                else
                {
                    if (!gnReady)
                    {
                        linSolveDumped(JTJ, g, hgn);
                        gnReady = true;
                    }
                    if ((!hgn) < trustRegion)
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
