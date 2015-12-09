#ifndef ABSOLUTENONCENTRALRANSACSOLVER
#define ABSOLUTENONCENTRALRANSACSOLVER

#include "calibrationPhotostation.h"
#include "levenmarq.h"

#include <vector>
#include <algorithm>

namespace corecvs
{
struct AbsoluteNonCentralRansacSolverParams
{
	double reprojectionInlierThreshold = 2.0;
	int fitIterations = 100;
	int maxIterations = 100000; // Big enough for P3P
};
class AbsoluteNonCentralRansacSolver : AbsoluteNonCentralRansacSolverParams
{
public:
	AbsoluteNonCentralRansacSolver(const corecvs::Photostation &ps, const std::vector<std::tuple<int, corecvs::Vector2dd, int, corecvs::Vector3dd>> &cloudMatches, const AbsoluteNonCentralRansacSolverParams &params = AbsoluteNonCentralRansacSolverParams()) : AbsoluteNonCentralRansacSolverParams(params), ps(ps), cloudMatches(cloudMatches)
	{
	}
	void run();
	void runInliersPNP();
	void runInliersRE();
	std::vector<int> getInliers();
	corecvs::Affine3DQ getBestHypothesis();
protected:
	void readParams(const double *in)
	{
		int argin = 0;
		for (int i = 0; i < 3; ++i)
			ps.location.shift[i] = in[argin++];
		for (int i = 0; i < 4; ++i)
			ps.location.rotor[i] = in[argin++];
		ps.location.rotor.normalise();
	}
	void writeParams(double *out)
	{
		for (int i = 0; i < 3; ++i)
			out[i] = ps.location.shift[i];
		for (int j = 0; j < 4; ++j)
			out[3 + j] = ps.location.rotor[j];
	}
	struct ReprojectionError : FunctionArgs
	{
		void operator() (const double in[], double out[])
		{
			solver->readParams(in);
			solver->computeReprojectionErrors(out);
		}
		ReprojectionError(AbsoluteNonCentralRansacSolver* solver) : FunctionArgs(7, solver->bestInlierCnt * 2), solver(solver)
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
		ReprojectionErrorNormalizer(AbsoluteNonCentralRansacSolver *solver): FunctionArgs(7, 7), solver(solver)
		{
		}
		AbsoluteNonCentralRansacSolver *solver;
	};
	void computeReprojectionErrors(double *out);

	std::vector<int> selectInliers(const corecvs::Affine3DQ &hypothesis);
	void getHypothesis();
	
	static const int SAMPLESIZE = 3;
	int idxs[SAMPLESIZE];
	corecvs::Affine3DQ bestHypothesis;
	int bestInlierCnt = 0;
	std::vector<int> inliers;
	std::vector<corecvs::Affine3DQ> hypothesis;
	corecvs::Photostation ps;
	std::vector<std::tuple<int, corecvs::Vector2dd, int, corecvs::Vector3dd>> cloudMatches;
};
}

#endif
