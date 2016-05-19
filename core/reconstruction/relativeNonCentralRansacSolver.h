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
    RelativeNonCentralRansacSolverSettings(size_t maxIterations = 400000, double inlierThreshold = 1.0, double gamma = 0.001)
        : maxIterations(maxIterations),
          inlierThreshold(inlierThreshold),
          gamma(gamma)
    {
    }
    size_t maxIterations;
    double inlierThreshold;

    enum class Restrictions
    {
        NONE,
        SCALE,
        SHIFT
    };

    Restrictions restrictions;
    corecvs::Vector3dd shift;
    double scale = 1.0;
    double gamma = 0.001;
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
    RelativeNonCentralRansacSolver(
          CameraFixture* query,
          const Affine3DQ firstTry
        , const MatchContainer &matchesRansac
        , const MatchContainer &matchesAll
        , const RelativeNonCentralRansacSolverSettings &settings = RelativeNonCentralRansacSolverSettings());
    ~RelativeNonCentralRansacSolver()
    {
        double total = totalEstiamte + totalSample + totalCheck;
        std::cout << "RNCRS timings: [Sample: " << totalSample / total * 100.0 << "% ][Estimate: " << totalEstiamte / total * 100.0 << "%][Check: " << totalCheck / total * 100.0 << "]" << std::endl;
    }
    size_t getInliersCount();
    void run();
    void makeTry(const Affine3DQ &hypo);
    void fit(double distanceGuess = 10.0);
    corecvs::Affine3DQ getBestHypothesis() const;
    std::vector<int> getBestInliers() const;
    double getGamma();
    int sampleSize() const { return restrictions == RelativeNonCentralRansacSolverSettings::Restrictions::SHIFT ? 3 : 6; }

private:
    struct Estimator
    {
        void operator() (const corecvs::BlockedRange<int> &r);
        Estimator(RelativeNonCentralRansacSolver *solver, double inlierThreshold, Restrictions restrictions, Vector3dd shift, double scale) : inlierThreshold(inlierThreshold), scale(scale), shift(shift), restrictions(restrictions), solver(solver)
        {
            rng = std::mt19937(std::random_device()());
            fundamentalsCache.resize(solver->fundamentalsCacheId.size());
            essentialsCache.resize(solver->fundamentalsCacheId.size());
            pluckerRef.resize(solver->sampleSize());
            pluckerQuery.resize(solver->sampleSize());
        }

        std::mt19937 rng;
        void sampleModel();
        void makeHypo();
        void selectInliers();

        size_t localMax = 0;
        int idxs[6];
        double inlierThreshold, scale;

        Vector3dd shift;
        Restrictions restrictions;

        RelativeNonCentralRansacSolver* solver;
        std::vector<corecvs::Affine3DQ> hypothesis;
        std::vector<std::vector<corecvs::Matrix33>> fundamentals;
        std::vector<std::vector<corecvs::EssentialDecomposition>> essentials;
        std::vector<std::pair<corecvs::Vector3dd, corecvs::Vector3dd>> pluckerRef, pluckerQuery;
        std::vector<corecvs::Matrix33> fundamentalsCache;
        std::vector<corecvs::EssentialDecomposition> essentialsCache;
    };
    struct ParallelEstimator
    {
        void operator() (const corecvs::BlockedRange<int> &r) const;
        ParallelEstimator(RelativeNonCentralRansacSolver* solver, int batch) : solver(solver), batch(batch)
        {
        }
        RelativeNonCentralRansacSolver *solver;
        int batch;
    };

#ifdef WITH_TBB
    tbb::mutex mutex;
#endif
    void accept(const Affine3DQ &hypo, const std::vector<int> &inliersNew);



    struct FunctorCost : corecvs::FunctionArgs
    {
        RelativeNonCentralRansacSolver *solver;
        FunctorCost(RelativeNonCentralRansacSolver* solver) : FunctionArgs(7, (int)solver->getInliersCount()), solver(solver)
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

    double nForGamma();
#if 1
    void buildDependencies();
    std::vector<int> dependencyList;
    std::vector<std::pair<WPP, WPP>> fundamentalsCacheId;
#endif

    std::mt19937 rng;
    static const int FEATURES_FOR_MODEL = 6;
    int maxInliers = 0;
    int nullInliers = 0;
    corecvs::Affine3DQ bestHypothesis;
//    std::vector<corecvs::Affine3DQ> currentHypothesis;
//    std::vector<int> currentScores;
//    std::vector<std::vector<int>> currentInliers;
    std::vector<int> bestInliers;

#if 1
#endif
    CameraFixture* query;
    MatchContainer matchesRansac, matchesAll;
    int batch = 100, batches = 64, usedEvals = 0;

    double totalSample = 0.0, totalEstiamte = 0.0, totalCheck = 0.0;
};

}

#endif // RELATIVENONCENTRALRANSACSOLVER
