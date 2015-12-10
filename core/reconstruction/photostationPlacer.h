#ifndef PHOTOSTATIONPLACER
#define PHOTOSTATIONPLACER

#include <string>
#include <vector>

#include "vector3d.h"
#include "calibrationPhotostation.h"
#include "reconstructionStructs.h"
#include "levenmarq.h"
#include "tbb/mutex.h"
#include <atomic>

namespace std
{
template<>
struct hash<std::tuple<int, int, int>>
{
	size_t operator() (const std::tuple<int, int, int>& t) const
	{
		return std::hash<int>()(std::get<0>(t)) ^
			  (std::hash<int>()(std::get<1>(t)) * 7) ^
			  (std::hash<int>()(std::get<2>(t)) * 31);
	}
};
template<>
struct hash<std::tuple<int, int, int, int>>
{
	size_t operator() (const std::tuple<int, int, int, int>& t) const
	{
		return std::hash<int>()(std::get<0>(t)) ^
			  (std::hash<int>()(std::get<1>(t)) * 7) ^
			  (std::hash<int>()(std::get<2>(t)) * 31) ^
			  (std::hash<int>()(std::get<3>(t) * 127));
	}
};
}

namespace corecvs
{

struct PhotostationPlacerFeatureParams
{
    std::string detector  = "SURF";
    std::string descriptor= "SURF";
    std::string matcher   = "ANN";
    double b2bThreshold = 0.9;
};

struct PhotostationPlacerEssentialFilterParams
{
    double b2bRansacP5RPThreshold = 0.8;
    double inlierP5RPThreshold = 3.0;
    int maxEssentialRansacIterations = 1000;
    double b2bRansacP6RPThreshold = 0.8;
};

struct PhotostationPlacerFeatureSelectionParams
{
    double inlierThreshold = 1.0;
    double trackInlierThreshold = 2.0;
    double pairCorrespondenceThreshold = 0.25;
    int nonLinearAfterAppend = 40;
    int nonLinearAfterAdd    = 40;
    int nonLinearFinal       =100;
};

class PhotostationPlacer : PhotostationPlacerFeatureParams, PhotostationPlacerEssentialFilterParams, PhotostationPlacerFeatureSelectionParams
{
public:
    void detectAll();
    void filterEssentialRansac();
    void estimateFirstPair();
    void estimatePair(int psA, int psB);
    corecvs::Quaternion detectOrientationFirst();
    void selectEpipolarInliers();
    void backprojectAll();
    void buildTracks(int psA, int psB, int psC);
    void fitLMedians();
    void appendPs();
	void appendTracks(const std::vector<int> &inlierIds, int ps);
	std::vector<std::vector<PointObservation__>> verify(const std::vector<PointObservation__> &pois);

    std::vector<std::tuple<int, corecvs::Vector2dd, int, corecvs::Vector3dd>> getPossibleTracks(int ps);

    std::vector<corecvs::Photostation> calibratedPhotostations;
    std::vector<std::vector<std::string>> images;
    std::vector<std::vector<std::vector<corecvs::Vector2dd>>> keyPoints;
	std::vector<std::vector<std::vector<corecvs::RGBColor>>> keyPointColors;
    std::vector<std::vector<std::vector<std::tuple<int, int, int, int, double>>>> matches, matchesCopy;
    std::vector<std::tuple<int, int, std::vector<int>>> pairInliers;
    std::vector<corecvs::Vector3dd> gpsData;
    std::vector<std::pair<corecvs::Vector3dd,corecvs::Vector3dd>> backprojected [6];

	std::vector<PointObservation__> tracks;
	std::unordered_map<std::tuple<int, int, int>, int> trackMap;
	int getReprojectionCnt();
	int getOrientationInputNum();
protected:
	void readOrientationParams(const double in[]);
	void writeOrientationParams(double out[]);
	void computeMedianErrors(double out[]);

	struct OrientationFunctor : corecvs::FunctionArgs
	{
		void operator() (const double in[], double out[])
		{
			placer->readOrientationParams(in);
			placer->computeMedianErrors(out);
		}
		OrientationFunctor(PhotostationPlacer *placer) : FunctionArgs(placer->getOrientationInputNum(), placer->getReprojectionCnt()), placer(placer)
		{
		}
		PhotostationPlacer* placer;
	};
	struct OrientationNormalizationFunctor : corecvs::FunctionArgs
	{
		OrientationNormalizationFunctor(PhotostationPlacer *placer) : FunctionArgs(placer->getOrientationInputNum(), placer->getOrientationInputNum()), placer(placer)
		{
		}
		void operator() (const double in[], double out[])
		{
			placer->readOrientationParams(in);
			placer->writeOrientationParams(out);
		}
		PhotostationPlacer* placer;
	};
private:

	double scoreFundamental(int psA, int camA, corecvs::Vector2dd ptA,
			                int psB, int camB, corecvs::Vector2dd ptB);
    int preplaced = 0, placed = 0;
    struct ParallelEssentialFilter
    {
        std::vector<std::tuple<int, int, int, int>> work;
        PhotostationPlacer* placer;
        static std::atomic<int> cntr;
        ParallelEssentialFilter(PhotostationPlacer* placer, std::vector<std::tuple<int,int,int,int>> &work) : placer(placer), work(work) {cntr = 0;}
        void operator() (const corecvs::BlockedRange<int> &r) const
        {
            for (int i = r.begin(); i < r.end(); ++i)
            {
                auto w = work[i];
                placer->filterEssentialRansac(std::get<0>(w), std::get<1>(w), std::get<2>(w), std::get<3>(w));
                cntr++;
                std::cout << ((double)cntr) / work.size() * 100.0 << "% complete" << std::endl;
            }
        }
    };
    void selectEpipolarInliers(int psA, int psB);
    void filterEssentialRansac(int psA, int camA, int psB, int camB);
    std::vector<std::tuple<int, corecvs::Vector2dd, int, corecvs::Vector2dd, double>> getPhotostationMatches(int psA, int psB);
    std::vector<std::tuple<corecvs::Vector2dd, corecvs::Vector2dd, double>> getCameraMatches(int psA, int camA, int psB, int camB);
    void remove(int psA, int psB, std::vector<int> idx);
    void remove(int psA, int camA, int psB, int camB, std::vector<int> idx);
    tbb::mutex mutex;
};
}

#endif
