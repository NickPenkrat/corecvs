#ifndef ASSERTS
#define ASSERTS
#endif

#include "gtest/gtest.h"

#include "global.h"
#include "polynomial.h"
#include "polynomialSolver.h"

#include <random>

const int DEFAULT_SEED = 777;
const int RNG_RETRIES = 131072;
const double REL_SOL_TOLERANCE = 1e-5;
const double COEFF_LIMIT = 1e2;
const double MAX_SCALE_LIMIT = 1e2;

TEST(PolynomialTest, testBasics)
{
    auto polyX = corecvs::Polynomial::X();
    ASSERT_TRUE(polyX.degree() == 1);
    ASSERT_TRUE((polyX * 2.0).degree() == 1);
    ASSERT_TRUE((polyX * corecvs::Polynomial::X()).degree() == 2);
    ASSERT_TRUE((polyX + 2.0).degree() == 1);
    ASSERT_TRUE((polyX - 2.0).degree() == 1);
    auto poly = (corecvs::Polynomial::X() - 1.0) *
                (corecvs::Polynomial::X() - 2.0) *
                (corecvs::Polynomial::X() - 3.0);
    std::cout << "Poly created: " << poly << std::endl;
    ASSERT_TRUE(poly.degree() == 3);
    ASSERT_NEAR(poly(1.0), 0.0, 1e-9);
    ASSERT_NEAR(poly(2.0), 0.0, 1e-9);
    ASSERT_NEAR(poly(3.0), 0.0, 1e-9);
    ASSERT_NEAR(poly(4.0), 6.0, 1e-9);
}

TEST(PolynomialTest, testFromRoots)
{
    std::vector<double> roots = {1.0, 2.0, 3.0, -5.0, 6.0, 7.0};
    auto poly = corecvs::Polynomial::FromRoots(roots);
    for (auto& r: roots)
        ASSERT_NEAR(poly(r), 0.0, 1e-9);
    ASSERT_FALSE(std::abs(poly(-1.0)) < 0.5);
}

TEST(PolynomialSolversTest, testPow1)
{
    std::mt19937 rng(DEFAULT_SEED);
    std::uniform_real_distribution<double> unif(-COEFF_LIMIT, COEFF_LIMIT);

    const int POW = 1;
    double coeff[2], roots[1];
    int rootsN;

    for (int i = 0; i < RNG_RETRIES; ++i)
    {
        coeff[0] = unif(rng);
        coeff[1] = unif(rng);

        rootsN = corecvs::PolynomialSolver::solve_imp<1>(coeff, roots);
        if (rootsN)
        {
            ASSERT_NEAR(coeff[0] + coeff[1] * roots[0], 0.0, REL_SOL_TOLERANCE * std::max(std::abs(coeff[0]), std::abs(coeff[1])));
        }
    }
    coeff[0] = 1e-11;
    coeff[1] = 1e-10;
    rootsN = corecvs::PolynomialSolver::solve_imp<1>(coeff, roots);
    ASSERT_TRUE(rootsN == 1);
    ASSERT_NEAR(corecvs::PolynomialSolver::evaluate(coeff, roots[0], 1), 0.0, REL_SOL_TOLERANCE);

    coeff[0] = 1e11;
    coeff[1] = 1e10;
    rootsN = corecvs::PolynomialSolver::solve_imp<1>(coeff, roots);
    ASSERT_TRUE(rootsN == 1);
    ASSERT_NEAR(corecvs::PolynomialSolver::evaluate(coeff, roots[0], 1), 0.0, REL_SOL_TOLERANCE);
}

TEST(PolynomialSolversTest, testPow2)
{
    std::mt19937 rng(DEFAULT_SEED);
    std::uniform_real_distribution<double> unif(-COEFF_LIMIT, COEFF_LIMIT);

    const int POW = 2;
    double coeff[3], roots[2];
    int rootsN;

    for (int i = 0; i < RNG_RETRIES; ++i)
    {
        coeff[0] = unif(rng);
        coeff[1] = unif(rng);
        coeff[2] = unif(rng);
        double mc = std::max(std::abs(coeff[0]), std::max(std::abs(coeff[1]), std::abs(coeff[2])));

        rootsN = corecvs::PolynomialSolver::solve_imp<2>(coeff, roots);
        if (rootsN)
        {
            ASSERT_NEAR(corecvs::PolynomialSolver::evaluate(coeff, roots[0], 2), 0.0, mc * REL_SOL_TOLERANCE);
            if (rootsN > 1)
                ASSERT_NEAR(corecvs::PolynomialSolver::evaluate(coeff, roots[1], 2), 0.0, mc * REL_SOL_TOLERANCE);
        }
    }
}

void coeffByRoots(std::vector<double> &coeff, std::vector<double> &roots)
{
    coeff.clear();
    coeff.resize(roots.size() + 2);
    coeff[0] = 1.0;
    for (auto& r: roots)
    {
        for (size_t i = roots.size(); i > 0; --i)
        {
            coeff[i] = (coeff[i - 1] - r * coeff[i]);
        }
        coeff[0] = -r * coeff[0];
    }
}

TEST(PolynomialSolversTest, testPowN)
{
    // (x - 1)(x - 2)(x - 3)
    double coeff3[] = {-6.0, 11.0, -6.0, 1.0};
    double roots3[3];

    int rcnt = (int)corecvs::PolynomialSolver::solve(coeff3, roots3, 3);

    ASSERT_TRUE(rcnt == 3);
    for (int i = 0; i < rcnt; ++i)
    {
        ASSERT_NEAR(corecvs::PolynomialSolver::evaluate(coeff3, roots3[i], 3), 0.0, REL_SOL_TOLERANCE);
    }

    std::mt19937 rng(DEFAULT_SEED);
    std::uniform_real_distribution<double> unif(-COEFF_LIMIT, COEFF_LIMIT);
    double maxdiff = 0.0;

    std::vector<double> coeff, roots, roots2;
    for (int i = 0; i < RNG_RETRIES; ++i)
    {
        int N = (rng() % 5) + 3;
        if (coeff.size() < N + 1)
        {
            coeff.resize(N + 1);
            roots.resize(N + 1);
        }
        for (int j = 0; j < N; ++j)
            roots[j] = unif(rng);
        roots.resize(N);
        roots2.resize(N);
        coeffByRoots(coeff, roots);

        int cnt = (int)corecvs::PolynomialSolver::solve(&coeff[0], &roots2[0], N);
        ASSERT_TRUE(cnt == N);
        std::sort(roots.begin(), roots.end());
        std::sort(roots2.begin(), roots2.end());
        for (size_t j = 0; j < N; ++j)
        {
            ASSERT_NEAR(roots[j], roots2[j], std::abs(roots[j]) * REL_SOL_TOLERANCE);
            if (std::abs(roots[j] - roots2[j]) / std::max(std::abs(roots[j]), 1.0))
            {
                maxdiff = std::abs(roots[j] - roots2[j]) / std::max(std::abs(roots[j]), 1.0);
            }
        }
    }
    std::cout << "MAX diff: " << maxdiff << std::endl;
}

TEST(PolynomialSolversTest, testPowN2)
{
    std::mt19937 rng(DEFAULT_SEED);
    std::uniform_real_distribution<double> unif(-COEFF_LIMIT, COEFF_LIMIT);
    for (int iter = 0; iter < RNG_RETRIES; ++iter)
    {
        int N = 3 + rng() % 4;
        std::vector<double> roots, roots2;
        for (int i = 0; i < N; ++i)
            roots.push_back(unif(rng));
        auto poly = corecvs::Polynomial::FromRoots(roots);
        corecvs::PolynomialSolver::solve(poly, roots2);
        ASSERT_TRUE(roots.size() == roots2.size());
        std::sort(roots.begin(), roots.end());
        std::sort(roots2.begin(), roots2.end());
        for (size_t i = 0; i < roots.size(); ++i)
            ASSERT_NEAR(roots[i], roots2[i], REL_SOL_TOLERANCE * std::abs(roots[i]));
    }
}
