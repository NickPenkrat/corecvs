#ifndef SCENEGENERATOR
#define SCENEGENERATOR

#include "reconstructionFixtureScene.h"

namespace corecvs
{

struct SceneGeneratorParams
{
    double rIn = 10.0, rOut = 500.0, sigmaProj = 0., gamma = 0.25, sigmaProjPOI = 1.0,
           poiMeasureLimit = 0.75;
    int N = 5, MPOI = 1000000, MPT = 5000;
    double rInPoi = 10.0, rOutPoi = 100.0;
    double R = 10.0;
    double sectorWidth = M_PI, sectorWidthPOI = M_PI;
    bool fSectorRnorm = false;

    SceneGeneratorParams(
        double rIn = 10.0,
        double rOut = 500.0,
        double sigmaProj = 0.,
        double gamma = 0.5,
        double sigmaProjPOI = 1.0,
        double poiMeasureLimit = 0.75,
        int N = 5,
        int MPOI = 100000,
        int MPT = 10000,
        double rInPoi = 10.0,
        double rOutPoi = 100.0,
        double R = 10.0,
        double sectorWidth = M_PI,
        double sectorWidthPOI = M_PI,
        bool fSectorRnorm = false) :
        rIn(rIn), rOut(rOut), sigmaProj(sigmaProj), gamma(gamma), sigmaProjPOI(sigmaProjPOI), poiMeasureLimit(poiMeasureLimit), N(N), MPOI(MPOI), MPT(MPT), rInPoi(rInPoi), rOutPoi(rOutPoi), R(R), sectorWidth(sectorWidth), sectorWidthPOI(sectorWidthPOI), fSectorRnorm(fSectorRnorm)
    {
    }


    friend std::ostream& operator<< (std::ostream& out, const SceneGeneratorParams &params)
    {
        out << "\t\tFeature r: (" << params.rIn << "; " << params.rOut <<") proj. stdev " << params.sigmaProj << " track.gamma = " << params.gamma << "; #:" << params.MPT << " azimuth distribution : " << (params.fSectorRnorm ? "Normal" : "Uniform") << std::endl
            << "\t\tPOI     r: (" << params.rInPoi << "; " << params.rOutPoi << ") proj. stdev " << params.sigmaProjPOI << " #:" << params.MPOI << std::endl
            << "\t\tFixtures: R: " << params.R << "; #:" << params.N << std::endl;
        return out;
    }
};

struct SceneGenerator : public SceneGeneratorParams
{
    SceneGenerator(SceneGeneratorParams params = SceneGeneratorParams()) : SceneGeneratorParams(params)
    {
    }
    void generateScene();
    void generatePoints();
    void generateFixtures();
    void generateMatches();
    Vector2dd generateError(std::mt19937 &rng, double std = 1.0, double threshold = 3.0);
    CameraModel generateCamera(int id);
    CameraFixture* generatePs(corecvs::Vector3dd pos, int id);
    ReconstructionFixtureScene* rfs =  nullptr;
};
};

#endif
