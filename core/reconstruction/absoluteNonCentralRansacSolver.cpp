#include "absoluteNonCentralRansacSolver.h"

#include "pnpSolver.h"
#include <random>

void corecvs::AbsoluteNonCentralRansacSolver::run()
{
	std::mt19937 rng;
    int N = (int)cloudMatches.size();
	
	for (int i = 0; i < maxIterations; ++i)
	{
		for(int rdy = 0; rdy < SAMPLESIZE;)
		{
			int idx = rng() % N;
			bool isOk = true;
			for (int j = 0; j < rdy && isOk; ++j)
				if (idxs[j] == idx)
					isOk = false;
			if (!isOk) continue;

			idxs[rdy++] = idx;
		}
		getHypothesis();
		int currentBest = 0;
		corecvs::Affine3DQ currentH;
		std::vector<int> currentInliers;
		for (auto& h: hypothesis)
		{
			auto ih = selectInliers(h);
			if (ih.size() < currentBest)
				continue;
            currentBest = (int)ih.size();
			currentInliers = ih;
			currentH = h;
		}

		if (currentBest <= bestInlierCnt)
			continue;

		bestInlierCnt = currentBest;
		inliers = currentInliers;
		bestHypothesis = currentH;
		std::cout << "P3P: " << bestInlierCnt << " " << ((double)bestInlierCnt) / ((double)N) * 100.0 << "%" << std::endl;
	}
}

void corecvs::AbsoluteNonCentralRansacSolver::runInliersRE()
{
	ReprojectionError err(this);
	ReprojectionErrorNormalizer norm(this);

	corecvs::LevenbergMarquardt lm;
	lm.f = &err;
	lm.normalisation = &norm;
	lm.maxIterations = fitIterations;
	lm.trace = false;//true;

	ps.location = bestHypothesis;
	std::vector<double> in(forcePosition ? 4 : 7), out(bestInlierCnt * 2);
	readParams(&in[0]);
	writeParams(&in[0]);
	auto res = lm.fit(in, out);
	readParams(&res[0]);
}

void corecvs::AbsoluteNonCentralRansacSolver::computeReprojectionErrors(double *out)
{
	int argout = 0;
	for (auto i: inliers)
	{
		auto t = cloudMatches[i];
		int cam = std::get<0>(t);
		auto pt = std::get<1>(t);
		auto ptw= std::get<3>(t);

		auto diff = (pt - ps.project(ptw, cam));
		out[argout++] = diff[0];
		out[argout++] = diff[1];
	}
}

std::vector<int> corecvs::AbsoluteNonCentralRansacSolver::selectInliers(const corecvs::Affine3DQ &hypo)
{
	ps.location = hypo;
    int M = (int)cloudMatches.size();
	std::vector<int> inliers;
	ps.location = hypo;
	if (forcePosition)
	    ps.location.shift = forcedPosition;
	for (int i = 0; i < M; ++i)
	{
		auto t = cloudMatches[i];
		int cam = std::get<0>(t);
		auto pt = std::get<1>(t);
		auto ptw= std::get<3>(t);

		double diff = !(pt - ps.project(ptw, cam));
		if (diff < reprojectionInlierThreshold && ps.isVisible(ptw, cam))
			inliers.push_back(i);
	}
	return inliers;
}

void corecvs::AbsoluteNonCentralRansacSolver::getHypothesis()
{
	ps.location.rotor = corecvs::Quaternion(0, 0, 0, 1);
	ps.location.shift = corecvs::Vector3dd(0, 0, 0);

	std::vector<corecvs::Vector3dd> centers, directions, points3d;
	for (int i: idxs)
	{
		auto t = cloudMatches[i];
		int cam = std::get<0>(t);
		auto pt = std::get<1>(t);
		auto ptw= std::get<3>(t);
		
		centers.push_back(ps.getRawCamera(cam).extrinsics.position);
		directions.push_back(ps.getRawCamera(cam).rayFromPixel(pt).a);
		points3d.push_back(ptw);
	}

	hypothesis = corecvs::PNPSolver::solvePNP(centers, directions, points3d);
}

void corecvs::AbsoluteNonCentralRansacSolver::runInliersPNP()
{
	ps.location.rotor = corecvs::Quaternion(0, 0, 0, 1);
	ps.location.shift = corecvs::Vector3dd(0, 0, 0);
    int N = (int)cloudMatches.size();

	std::vector<corecvs::Vector3dd> centers, directions, points3d;
	for (int i: inliers)
	{
		auto t = cloudMatches[i];
		int cam = std::get<0>(t);
		auto pt = std::get<1>(t);
		auto ptw= std::get<3>(t);
		
		centers.push_back(ps.getRawCamera(cam).extrinsics.position);
		directions.push_back(ps.getRawCamera(cam).rayFromPixel(pt).a);
		points3d.push_back(ptw);
	}

	hypothesis = corecvs::PNPSolver::solvePNP(centers, directions, points3d);

	int currentBest = 0;
	corecvs::Affine3DQ currentH;
	std::vector<int> currentInliers;
	for (auto& h: hypothesis)
	{
		auto ih = selectInliers(h);
		if (ih.size() < currentBest)
			continue;
        currentBest    = (int)ih.size();
		currentInliers = ih;
		currentH       = h;
	}

	// even if total solution has less inliers it is better
	//if (currentBest < bestInlierCnt)
	//	return;

	bestInlierCnt = currentBest;
	inliers = currentInliers;
	bestHypothesis = currentH;
	std::cout << "PNP: " << bestInlierCnt << " " << ((double)bestInlierCnt) / ((double)N) * 100.0 << "%" << std::endl;
}


std::vector<int> corecvs::AbsoluteNonCentralRansacSolver::getInliers()
{
	return inliers;
}

corecvs::Affine3DQ corecvs::AbsoluteNonCentralRansacSolver::getBestHypothesis()
{
	return bestHypothesis;
}
