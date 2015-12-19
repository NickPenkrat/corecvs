#ifndef PHOTOSTATIONPLACER
#define PHOTOSTATIONPLACER

#include <string>
#include <vector>

#include "vector3d.h"
#include "calibrationPhotostation.h"
#include "tbb/mutex.h"
#include <atomic>

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
    double inlierP5RPThreshold = 5.0;
    int maxEssentialRansacIterations = 10000;
    double b2bRansacP6RPThreshold = 0.8;
};

struct PhotostationPlacerFeatureSelectionParams
{
    double inlierThreshold = 1.0;
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

    std::vector<corecvs::Photostation> calibratedPhotostations;
    std::vector<std::vector<std::string>> images;
    std::vector<std::vector<std::vector<corecvs::Vector2dd>>> keyPoints;
    std::vector<std::vector<std::vector<std::tuple<int, int, int, int, double>>>> matches, matchesCopy;
    std::vector<std::tuple<int, int, std::vector<int>>> pairInliers;
    std::vector<corecvs::Vector3dd> gpsData;
    std::vector<corecvs::Vector3dd> backprojected;

private:
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
