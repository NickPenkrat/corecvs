#ifndef SCENEGENERATOR
#define SCENEGENERATOR

#include "reconstructionFixtureScene.h"

namespace corecvs
{
struct SceneGenerator
{
	void generateScene();
	void generatePoints();
	void generateFixtures();
	void generateMatches();
	CameraModel generateCamera(int id);
	CameraFixture* generatePs(corecvs::Vector3dd pos, int id);
	double rIn = 8.0, rOut = 1000.0, sigmaProj = 0.00001, gamma = 0.5, sigmaProjPOI = 1.0,
		   poiMeasureLimit = 0.75;
	int N = 5, M = 10000;
	double R = 10.0;
	ReconstructionFixtureScene* rfs =  nullptr;
};
};

#endif
