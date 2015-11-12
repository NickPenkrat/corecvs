#ifndef RECONSTRUCTIONSOLVER_H
#define RECONSTRUCTIONSOLVER_H

// TODO: refine this list
#include <vector>
#include <string>
#include <sstream>
#include <regex>
#include <fstream>
#include <unordered_map>
#include <unordered_set>
#include <cstdio>
#include <algorithm>
#include <iomanip>
#include <chrono>
#include <random>
#include "calibrationHelpers.h"
#include "calibrationJob.h"
#include "calibrationLocation.h"
#include "mesh3d.h"
#include "jsonSetter.h"
#include "jsonGetter.h"

#include "imageKeyPoints.h"
#include "reconstructionStructs.h"

#include "vector3d.h"
#include "vector2d.h"
#include "bufferReaderProvider.h"

#include "openCvFileReader.h"
#include "featureMatchingPipeline.h"
#include "openCvDescriptorExtractorWrapper.h"
#include "openCvFeatureDetectorWrapper.h"
#include "openCvDescriptorMatcherWrapper.h"
#include "multicameraTriangulator.h"
#include "undirectedGraph.h"
#include "multiPhotostationScene.h"
#include "essentialEstimator.h"

// TODO: make this stuff reconstruction params
#define POI_ONLY
#define NOUPD
#define METERS

struct ReconstructionParameters
{
    DetectorType   detector   = "SURF";
    DescriptorType descriptor = "SURF";
    MatcherType    matcher    = "ANN";//"ANN";

    template<typename V>
    void accept(V &visitor)
    {
        visitor.visit(detector,   "SURF", "detector");
        visitor.visit(descriptor, "SURF", "descriptor");
        visitor.visit(matcher,    "ANN",  "matcher");
    }
};

struct ReconstructionJob : ReconstructionParameters
{
    CalibrationJob         calibrationData;
    MultiPhotostationScene scene;

    int getOutputNum() const;
    int getInputNum() const;

    void getScaler(corecvs::Vector3dd &mean, corecvs::Vector3dd &scale);

    void writeParams(double out[], corecvs::Vector3dd mean = corecvs::Vector3dd(0.0, 0.0, 0.0), corecvs::Vector3dd scale = corecvs::Vector3dd(1.0, 1.0, 1.0)) const;


    void readParams(const double in[], corecvs::Vector3dd mean = corecvs::Vector3dd(0.0, 0.0, 0.0), corecvs::Vector3dd scale = corecvs::Vector3dd(1.0, 1.0, 1.0));

    struct OptimizationFunctor : public corecvs::FunctionArgs
    {
    private:
        ReconstructionJob* rJob;
        bool saturated;
        double saturationThreshold;
        corecvs::Vector3dd mean, scale;
        bool angleError = true;
    public:
        OptimizationFunctor(decltype(rJob) rJob, bool saturated = false, double saturationThreshold = 10, corecvs::Vector3dd mean = corecvs::Vector3dd(0.0, 0.0, 0.0), corecvs::Vector3dd scale = corecvs::Vector3dd(1.0, 1.0, 1.0), bool angleError=true/*,corecvs::Matrix22 covariation = corecvs::Matrix22(1.0, 0.0, 0.0, 1.0)*/) : FunctionArgs(rJob->getInputNum(), saturated || angleError ? rJob->getOutputNum() / 2 : rJob->getOutputNum()), rJob(rJob), saturated(saturated), saturationThreshold(saturationThreshold), mean(mean), scale(scale), angleError(angleError) //, covariation(covariation)
        {
        }
        void operator() (const double in[], double out[]);
    };

    void undistortAll(bool singleDistortion = true);
    void solve(bool angleError = true);
    void fill(std::unordered_map<std::string, corecvs::Affine3DQ> &data, int psLocationCnt = 5);

    template <typename V>
    void accept(V &visitor)
    {
        ReconstructionParameters::accept(visitor);
        visitor.visit(scene,           MultiPhotostationScene(), "scene");
        visitor.visit(calibrationData, CalibrationJob(), "calibrationData");
    }

    struct ParallelUndistortionMapEstimator
    {
        void operator() (const corecvs::BlockedRange<int> &r) const;

        std::vector<corecvs::DisplacementBuffer> *transformations;
        CalibrationJob *calibrationJob;
        Photostation *photostation;
        
        ParallelUndistortionMapEstimator(decltype(transformations) transformations, CalibrationJob* calibrationJob, Photostation* photostation) : transformations(transformations), calibrationJob(calibrationJob), photostation(photostation)
        {
        }
    };

    struct ParallelUndistortionCalculator
    {
        void operator() (const corecvs::BlockedRange<int> &r) const;

        std::vector<corecvs::DisplacementBuffer> *transformations;
        std::vector<CameraObservation> *observations;
        Photostation *photostation;
        
        ParallelUndistortionCalculator(decltype(observations) observations, decltype(transformations) transformations, Photostation* photostation)
            : transformations(transformations), observations(observations), photostation(photostation)
        {
        }
    };
};


#endif
