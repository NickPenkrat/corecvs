#ifndef ABSOLUTENONCENTRALRANSACSOLVER
#define ABSOLUTENONCENTRALRANSACSOLVER

//#include "calibrationPhotostation.h"
#include "levenmarq.h"
#include "cameraFixture.h"
#include "fixtureScene.h"
#include "tbbWrapper.h"
#ifdef WITH_TBB
#include <tbb/mutex.h>
#endif

#include <vector>
#include <algorithm>
#include <random>

namespace corecvs
{
struct AbsoluteNonCentralRansacSolverParams
{
    double reprojectionInlierThreshold = 2.0;
    int fitIterations = 100;
    int maxIterations = 100000; // Big enough for P3P
    bool forcePosition = false;
    corecvs::Vector3dd forcedPosition = corecvs::Vector3dd(0, 0, 0);
};
class AbsoluteNonCentralRansacSolver : public AbsoluteNonCentralRansacSolverParams
{
public:
    AbsoluteNonCentralRansacSolver(corecvs::CameraFixture *ps, const std::vector<std::tuple<FixtureCamera*, corecvs::Vector2dd, corecvs::Vector3dd, SceneFeaturePoint*, int>> &cloudMatches, const AbsoluteNonCentralRansacSolverParams &params = AbsoluteNonCentralRansacSolverParams()) : AbsoluteNonCentralRansacSolverParams(params), ps(ps), cloudMatches(cloudMatches)
    {
    }
    void run();
    void runInliersPNP();
    void runInliersRE();
    std::vector<int> getInliers();
    corecvs::Affine3DQ getBestHypothesis();
//protected:
#ifdef WITH_TBB
    tbb::mutex mutex;
#endif
    struct Estimator
    {
        void operator() (const corecvs::BlockedRange<int> &r);
        Estimator(AbsoluteNonCentralRansacSolver *solver, double inlierThreshold) :
            inlierThreshold(inlierThreshold),
            solver(solver)
        {
            rng = std::mt19937(std::random_device()());
        }
        std::mt19937 rng;
        void sampleModel();
        void makeHypo();
        void selectInliers();
        int localMax = 0;         /*< this probably needs to be size_t */
        static const int SAMPLESIZE = 3;
        int idxs[SAMPLESIZE];
        double inlierThreshold;
        AbsoluteNonCentralRansacSolver *solver;
        std::vector<corecvs::Affine3DQ> hypothesis;
    };

    struct ParallelEstimator
    {
        void operator() (const corecvs::BlockedRange<int> &r) const;
        ParallelEstimator(AbsoluteNonCentralRansacSolver *solver, int batch) : solver(solver), batch(batch)
        {
        }
        AbsoluteNonCentralRansacSolver *solver;
        int batch;
    };

    void accept(const corecvs::Affine3DQ &hypo, std::vector<int> &inliersNew, double inliersQuality)
    {
#ifdef WITH_TBB
        tbb::mutex::scoped_lock lock(mutex);
#endif
        if (inliersNew.size() < inliers.size() || (inliersQuality > inlierQuality && inliers.size() == inliersNew.size()))
            return;
        inliers = inliersNew;
        inlierQuality = inliersQuality;
        bestHypothesis = hypo;
        bestInlierCnt = (int)inliers.size();
        int N = (int)cloudMatches.size();
        std::cout << "P3P: " << bestInlierCnt << " " << ((double)bestInlierCnt) / ((double)N) * 100.0 << "%" << std::endl;
    }

    void readParams(const double *in)
    {
        int argin = 0;
        if (!forcePosition)
            for (int i = 0; i < 3; ++i)
                ps->location.shift[i] = in[argin++];
        for (int i = 0; i < 4; ++i)
            ps->location.rotor[i] = in[argin++];
        ps->location.rotor.normalise();
    }
    void writeParams(double *out)
    {
        int argout = 0;
        if (!forcePosition)
            for (int i = 0; i < 3; ++i)
                out[argout++] = ps->location.shift[i];
        for (int j = 0; j < 4; ++j)
            out[argout++] = ps->location.rotor[j];
    }
    struct ReprojectionError : FunctionArgs
    {
        void operator() (const double in[], double out[])
        {
            solver->readParams(in);
            solver->computeReprojectionErrors(out);
        }
        ReprojectionError(AbsoluteNonCentralRansacSolver* solver) : FunctionArgs(solver->forcePosition ? 4 : 7, solver->bestInlierCnt * 2), solver(solver)
        {
        }
        AbsoluteNonCentralRansacSolver *solver;
    };

    struct ReprojectionErrorNormalizer : FunctionArgs
    {
        void operator() (const double in[], double out[])
        {
            solver->readParams(in);
            solver->writeParams(out);
        }
        ReprojectionErrorNormalizer(AbsoluteNonCentralRansacSolver *solver): FunctionArgs(solver->forcePosition ? 4 : 7, solver->forcePosition ? 4 : 7), solver(solver)
        {
        }
        AbsoluteNonCentralRansacSolver *solver;
    };
    void computeReprojectionErrors(double *out);

    std::vector<int> selectInliers(const corecvs::Affine3DQ &hypothesis);
    void getHypothesis();

    corecvs::Affine3DQ bestHypothesis;
    int bestInlierCnt = 0;
    double inlierQuality = 0.0;
    std::vector<int> inliers;
    corecvs::CameraFixture *ps;
    std::vector<std::tuple<FixtureCamera*, corecvs::Vector2dd, corecvs::Vector3dd, SceneFeaturePoint*, int>> cloudMatches;
    std::vector<corecvs::Affine3DQ> hypothesis;
    int batch = 100;
    int batches = 64;
    int usedEvals = 0;
    double gamma = 0.001;
    double nForGamma();
public:
    ~AbsoluteNonCentralRansacSolver()
    {
        std::cout << "P3P: allowed: " << maxIterations << " used: " << usedEvals << std::endl;
    }
};
}

#endif
