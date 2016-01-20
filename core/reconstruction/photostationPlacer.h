#ifndef PHOTOSTATIONPLACER
#define PHOTOSTATIONPLACER

#include <string>
#include <vector>
#include <atomic>

#include "vector3d.h"
#include "calibrationPhotostation.h"
#include "reconstructionStructs.h"
#include "reconstructionFixtureScene.h"
#include "levenmarq.h"
#include "mesh3d.h"

#ifdef WITH_TBB
#include "tbb/mutex.h"
#endif

#include "typesafeBitmaskEnums.h"

enum class PhotostationPlacerOptimizationType
{
    NON_DEGENERATE_ORIENTATIONS = 1, // Orientations of all cameras except first
    DEGENERATE_ORIENTATIONS = 2,     // Orientation of first camera
    NON_DEGENERATE_TRANSLATIONS = 4, // Translations of all cameras except first (TODO: Clarify if for noncentral camera we would like to fix "scale")
    DEGENERATE_TRANSLATIONS = 8,     // Translation of first camera
    FOCALS = 16,                     // Camera focals in multicamera
    PRINCIPALS = 32,                 // Camera principals in multicamera
    POINTS = 64,                     // 3D points
};
enum class PhotostationPlacerOptimizationErrorType
{
    REPROJECTION,                    // Reprojection error
    ANGULAR,                         // Angular error
    CROSS_PRODUCT,                    // Cross product error
    RAY_DIFF
};

/*
 * Parameters are stored in the following way (section is omitted depending on PhotostationPlacerOptimizationType)
 * N - number of "preplaced" photostations
 * M - number of points
 * M'- number of projections
 * K - number of cameras in photostation
 *
 * 4            Orientation of first multicamera       DEGENERATE_ORIENTATIONS
 * 4 x (N - 1)  Orientations of multicameras           NON_DEGENERATE_ORIENTATIONS
 * 3            Translation of first multicamera       DEGENERATE_TRANSLATIONS
 * 3 x (N - 1)  Translations of multicameras           NON_DEGENERATE_TRANSLATIONS
 * 1 x K        Focal lengths of cameras               FOCALS
 * 2 x K        Principal point projections of cameras PRINCIPALS
 * 3 x M        3D points                              POINTS
 *
 * Output parameters are stored sequentially as all projections of all 3d points
 * 2 x M'       Reprojections                          ANGULAR
 * 1 x M'       Angles between rays                    REPROJECTION
 * 3 x M'       Ray cross products                     CROSS_PRODUCT
 * 2 x M'       Ray differences                        RAY_DIFF
 */

template<>
struct is_bitmask<PhotostationPlacerOptimizationType> : std::true_type {};


namespace corecvs
{

struct PhotostationPlacerFeatureParams
{
    std::string detector  = "SURF";
    std::string descriptor= "SURF";
    std::string matcher   = "ANN";
    double b2bThreshold = 0.9;

    template<typename V>
    void accept(V &v)
    {
        v.visit(detector,  std::string("SURF"), "Feature detector");
        v.visit(descriptor, std::string("SURF"), "Feature descriptor");
        v.visit(matcher,   std::string("ANN"),  "Feature matcher");
        v.visit(b2bThreshold, 0.9, "Best-2nd best ratio threshold");
    }
};

struct PhotostationPlacerEssentialFilterParams
{
    double b2bRansacP5RPThreshold = 0.8;
    double inlierP5RPThreshold = 5.0;
    int maxEssentialRansacIterations = 1000;
    double b2bRansacP6RPThreshold = 0.8;

    template<typename V>
    void accept(V &v)
    {
        v.visit(b2bRansacP5RPThreshold, 0.8, "Best-2nd best essential estimator threshold");
        v.visit(inlierP5RPThreshold, 5.0, "Inlier threshold");
        v.visit(maxEssentialRansacIterations, 1000, "Maximal essential estimator rounds");
        v.visit(b2bRansacP6RPThreshold, 0.8, "Best-2nd best relative pose estimator threshold");
    }
};

struct PhotostationPlacerFeatureSelectionParams
{
    double inlierThreshold = 1.0;
    double trackInlierThreshold = 3.0;
    double pairCorrespondenceThreshold = 0.25;
    double distanceLimit =1000.0;
//    int nonLinearAfterAppend = 40;
//    int nonLinearAfterAdd    = 40;
//    int nonLinearFinal       =100;
//    bool tuneFocal = false;

    template<typename V>
    void accept(V &v)
    {
        v.visit(inlierThreshold, 1.0, "Inlier threshold");
        v.visit(trackInlierThreshold, 3.0, "Track append threshold");
        v.visit(pairCorrespondenceThreshold, 0.25, "Correspondence threshold");
        v.visit(distanceLimit, 1000.0, "Track distance limit");
    }
};

struct PhotostationPlacerParams
{
    bool forceGps = true;
    PhotostationPlacerOptimizationType optimizationParams = 
        PhotostationPlacerOptimizationType::NON_DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::DEGENERATE_ORIENTATIONS |
        PhotostationPlacerOptimizationType::POINTS;// | PhotostationPlacerOptimizationType::FOCALS;
    PhotostationPlacerOptimizationErrorType errorType = PhotostationPlacerOptimizationErrorType::RAY_DIFF;

    template<typename V>
    void accept(V &v)
    {
        v.visit(forceGps, true, "Enforce gps locations");
        auto vv = static_cast<typename std::underlying_type<PhotostationPlacerOptimizationType>::type>(optimizationParams);
        v.visit(vv, 0, "Optimization type");
        optimizationParams = static_cast<PhotostationPlacerOptimizationType>(vv);
        auto vvv = static_cast<typename std::underlying_type<PhotostationPlacerOptimizationErrorType>::type>(errorType);
        v.visit(vvv, 0, "Error type");
        errorType = static_cast<PhotostationPlacerOptimizationErrorType>(vvv);
    }
};

class PhotostationPlacer : PhotostationPlacerFeatureParams, PhotostationPlacerEssentialFilterParams, PhotostationPlacerFeatureSelectionParams, PhotostationPlacerParams
{
public:
	std::unordered_map<std::tuple<int, int, int>, int> trackMap;
    ReconstructionFixtureScene* scene;
#if 0
    void fullRun();
    corecvs::Mesh3D dumpMesh(const std::string &filename, bool drawTracks = false, bool center = true);
#endif
    void detectAll();
    bool initalize();
    bool initGPS();
    bool initNONE();
    bool initSTATIC();
    bool initFIXED();
    //void filterEssentialRansac();
#if 0
    void estimateFirstPair();
    void estimatePair(int psA, int psB);
    corecvs::Quaternion detectOrientationFirst();
    void selectEpipolarInliers();
    void backprojectAll();
    void buildTracks(int psA, int psB, int psC);
    void fit(bool tuneFocal);
    void fit(const PhotostationPlacerOptimizationType& optimizationSet = PhotostationPlacerOptimizationType::NON_DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::DEGENERATE_ORIENTATIONS | PhotostationPlacerOptimizationType::POINTS | PhotostationPlacerOptimizationType::FOCALS | PhotostationPlacerOptimizationType::PRINCIPALS, int num = 100);
    void appendPs();
	void appendTracks(const std::vector<int> &inlierIds, int ps);
	std::vector<std::vector<PointObservation__>> verify(const std::vector<PointObservation__> &pois);
    std::vector<PointObservation__> projectToAll(const std::vector<PointObservation__> &pois);

    std::vector<std::tuple<int, corecvs::Vector2dd, int, corecvs::Vector3dd>> getPossibleTracks(int ps);
#endif
#if 0
    std::vector<corecvs::Photostation> calibratedPhotostations;
    std::vector<std::vector<std::string>> images;
    std::vector<std::vector<std::vector<corecvs::Vector2dd>>> keyPoints;
	std::vector<std::vector<std::vector<corecvs::RGBColor>>> keyPointColors;
    std::vector<std::vector<std::vector<std::tuple<int, int, int, int, double>>>> matches, matchesCopy;
#if 0
    std::vector<std::tuple<int, int, std::vector<int>>> pairInliers;
#endif
    std::vector<PhotostationInitialization> psInitData;
#if 0
    std::vector<std::pair<corecvs::Vector3dd,corecvs::Vector3dd>> backprojected [6];
#endif
	std::vector<PointObservation__> tracks;
	std::unordered_map<std::tuple<int, int, int>, int> trackMap;
#endif
#if 0
	int getMovablePointCount();
	int getReprojectionCnt();
	int getOrientationInputNum();
	int getErrorComponentsPerPoint();
	void getErrorSummary(PhotostationPlacerOptimizationErrorType errorType);
	void getErrorSummaryAll();
#endif
protected:
#if 0
	void readOrientationParams(const double in[]);
	void writeOrientationParams(double out[]);
	void computeMedianErrors(double out[], const std::vector<int> &idxs);
    std::vector<std::vector<int>> getDependencyList();
    std::vector<std::pair<int, int>> revDependency; // track, projection

    struct ParallelErrorComputator
    {
        void operator() (const corecvs::BlockedRange<int> &r) const;
        ParallelErrorComputator(PhotostationPlacer* placer, const std::vector<int> &idxs, double* output) : placer(placer), idxs(idxs), output(output)
        {
        }
        PhotostationPlacer* placer;
        std::vector<int> idxs;
        double* output;
    };
	struct OrientationFunctor : corecvs::SparseFunctionArgs
	{
		void operator() (const double in[], double out[], const std::vector<int> &idxs)
		{
			placer->readOrientationParams(in);
			placer->computeMedianErrors(out, idxs);
		}
		OrientationFunctor(PhotostationPlacer *placer) : SparseFunctionArgs(placer->getOrientationInputNum(), placer->getReprojectionCnt(), placer->getDependencyList()), placer(placer)
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
public:
#if 0
	template<typename V>
	void accept(V &v)
    {
        v.visit(calibratedPhotostations, "Photostations");
        v.visit(keyPoints, "Keypoints");
        v.visit(keyPointColors, "Keypoint colors");
        visitTM("Matches", matches, v);
        // Here we hope that reconstruction is not interrupted in the middle of smth requiring
        // copy of matches
        //visitTM("Matches copy", matchesCopy, v);
        v.visit(psInitData, "Initialization");
        v.visit(tracks, "Tracks");
        v.visit(placed, 0, "Placed");
        v.visit(preplaced, 0, "Preplaced");
        v.visit(images, "images");
        v.visit((PhotostationPlacerFeatureParams&)*this, "Feature detection params");
        v.visit((PhotostationPlacerEssentialFilterParams&)*this, "Essential filtering params");
        v.visit((PhotostationPlacerFeatureSelectionParams&)*this, "Feature selection params");
        v.visit((PhotostationPlacerParams&)*this, "Reconstruction params");
    }

    template<typename V>
    void visitTM(const char* name, std::vector<std::vector<std::vector<std::tuple<int, int, int, int, double>>>> &tm, V &v)
    {
        std::vector<std::vector<std::vector<std::vector<double>>>> moo;
        for (auto& vvt: tm)
        {
            std::vector<std::vector<std::vector<double>>> foo;
            for (auto& vt: vvt)
            {
                std::vector<std::vector<double>> boo;
                for (auto& t: vt)
                    boo.push_back(std::vector<double>(
                        {(double)std::get<0>(t),
                         (double)std::get<1>(t),
                         (double)std::get<2>(t),
                         (double)std::get<3>(t),
                         (double)std::get<4>(t)}));
                foo.emplace_back(boo);
            }
            moo.emplace_back(foo);
        }
        v.visit(moo, name);
        tm.clear();
        for (auto& vvt: moo)
        {
            std::vector<std::vector<std::tuple<int, int, int, int, double>>> foo;
            for (auto& vt: vvt)
            {
                std::vector<std::tuple<int, int, int, int, double>> boo;
                for (auto& t: vt)
                {
                    boo.emplace_back(t[0], t[1], t[2], t[3], t[4]);
                }
                foo.emplace_back(boo);
            }
            tm.emplace_back(foo);
        }
    }
    template<typename V>
    void visitTrackmap(const char* name, V &v)
    {
        std::vector<std::vector<int>> trackmap;
        for (auto& el: trackMap)
            trackmap.emplace_back(std::vector<int>({std::get<0>(el.first), std::get<1>(el.first), std::get<2>(el.first), el.second}));
        v.visit(trackmap, "Track map");
        trackMap.clear();
        for (auto& el: trackmap)
            trackMap[std::make_tuple(el[0], el[1], el[2])] = el[3];
    }
#endif
private:

	double scoreFundamental(int psA, int camA, corecvs::Vector2dd ptA,
			                int psB, int camB, corecvs::Vector2dd ptB);
    int preplaced = 0, placed = 0;
    struct ParallelEssentialFilter
    {
        PhotostationPlacer* placer;
        std::vector<std::tuple<int, int, int, int>> work;
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
#endif
#ifdef WITH_TBB
    tbb::mutex mutex;
#endif
};
}

#endif
