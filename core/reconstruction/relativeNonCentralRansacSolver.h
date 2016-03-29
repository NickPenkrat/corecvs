#ifndef RELATIVENONCENTRALRANSACSOLVER
#define RELATIVENONCENTRALRANSACSOLVER

#include <vector>
#include <random>

#include "reconstructionFixtureScene.h"
#include "calibrationPhotostation.h"
#include "vector2d.h"
#include "levenmarq.h"

namespace corecvs
{

struct RelativeNonCentralRansacSolverSettings
{
    RelativeNonCentralRansacSolverSettings(size_t maxIterations = 500000, double inlierThreshold = 1.0)
        : maxIterations(maxIterations)
        , inlierThreshold(inlierThreshold)
    {
    }
    size_t maxIterations = 1000000LLU;
    double inlierThreshold = 1.0;
};


class RelativeNonCentralRansacSolver : public RelativeNonCentralRansacSolverSettings
{
public:
    typedef std::vector<std::tuple<WPP, corecvs::Vector2dd, WPP, corecvs::Vector2dd>> MatchContainer;

    RelativeNonCentralRansacSolver(
          CameraFixture* query
        , const MatchContainer &matchesRansac
        , const MatchContainer &matchesAll
        , const RelativeNonCentralRansacSolverSettings &settings = RelativeNonCentralRansacSolverSettings());

    size_t getInliersCount();
    void run();
    void fit(double distanceGuess = 10.0);
    corecvs::Affine3DQ getBestHypothesis() const;
    std::vector<int> getBestInliers() const;
    double getGamma();

private:
    struct FunctorCost : corecvs::FunctionArgs
    {
        RelativeNonCentralRansacSolver *solver;
        FunctorCost(RelativeNonCentralRansacSolver* solver) : FunctionArgs(7, solver->getInliersCount()), solver(solver)
        {}

        void operator() (const double in[], double out[])
        {
            solver->readParams(in);
            solver->computeError(out);
        }
    };

    struct FunctorCostNorm : corecvs::FunctionArgs
    {
        RelativeNonCentralRansacSolver *solver;
        FunctorCostNorm(RelativeNonCentralRansacSolver* solver) : FunctionArgs(7, 7), solver(solver)
        {}

        void operator() (const double in[], double out[])
        {
            solver->readParams(in);
            solver->writeParams(out);
        }
    };

    void readParams(const double in[]);
    void writeParams(double out[]);
    void computeError(double out[]);
    void updateInliers();

    void sampleRays();
    void estimatePose();
    void scoreCurrent();
    void selectBest();

    std::mt19937 rng;
    static const int FEATURES_FOR_MODEL = 6;
    int maxInliers = 0;
    int nullInliers = 0;
    corecvs::Affine3DQ bestHypothesis;
    std::vector<std::vector<corecvs::Matrix33>> fundamentals;
    std::vector<std::vector<corecvs::EssentialDecomposition>> essentials;
    std::vector<std::pair<corecvs::Vector3dd, corecvs::Vector3dd>> pluckerRef, pluckerQuery;
    std::vector<corecvs::Affine3DQ> currentHypothesis;
    std::vector<int> currentScores;
    std::vector<std::vector<int>> currentInliers;
    std::vector<int> bestInliers;

    CameraFixture* query;
    MatchContainer matchesRansac, matchesAll;
};

}

#endif // RELATIVENONCENTRALRANSACSOLVER
