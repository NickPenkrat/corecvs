#ifndef FLATPATTERNCALIBRATOR
#define FLATPATTERNCALIBRATOR

#include <vector>

#include "homographyReconstructor.h"
#include "levenmarq.h"
#include "lineDistortionEstimatorParameters.h"

#include "calibrationPhotostation.h"

// In order to get 3-dof rotation, we should penalize for quaternion norm
// The unclear part is it's weight
//#define PENALIZE_QNORM

/*
 * This class performs single-camera calibration from multiple views of flat pattern
 * using non-linear optimization with initialization via technique
 * described in Zhengyou Zhang A Flexible New Technique for Camera Calibration
 */
struct FlatPatternCalibrator
{
public:
    FlatPatternCalibrator(const CameraConstraints constraints = CameraConstraints::NONE, const PinholeCameraIntrinsics lockParams = PinholeCameraIntrinsics(), const LineDistortionEstimatorParameters distortionEstimatorParams = LineDistortionEstimatorParameters(), const double lockFactor = 1.0);

    // Add 2d-3d correspondences and initial guess for camera location
    // TODO: add check for pattern planarity and [maybe] support of other planes than z=0
	void addPattern(const PatternPoints3d &patternPoints, const CameraLocationData &position = CameraLocationData());

    // Runs (pre-) solver
	void solve(bool runPresolver = true,bool runLM = false, int LMiterations = 1000);

    PinholeCameraIntrinsics getIntrinsics();
    LensDistortionModelParameters getDistortion();

    std::vector<CameraLocationData> getExtrinsics();
    
    // Returns RMSE for reprojection
    double getRmseReprojectionError();

    // Returns full error
    void getFullReprojectionError(double out[]);

    int getInputNum() const;
    int getOutputNum() const;
    double factor = 1.0;
private:
    size_t K, N;

    void enforceParams();

    // Presolver
	void solveInitialIntrinsics();
	void solveInitialExtrinsics();
    void computeHomographies();
    void computeAbsoluteConic();
    void extractIntrinsics();

    // LM-solver
    void refineGuess(int LMiterations);

    void readParams(const double in[]);
    void writeParams(double out[]);

    struct LMCostFunction : public corecvs::FunctionArgs
    {
        LMCostFunction(FlatPatternCalibrator *calibrator)
            : FunctionArgs(calibrator->getInputNum(), calibrator->getOutputNum()), calibrator(calibrator)
        {
        }
        void operator()(const double in[], double out[]);
        FlatPatternCalibrator *calibrator;
    };

    corecvs::Vector absoluteConic;

    std::vector<corecvs::Matrix33> homographies;
	std::vector<PatternPoints3d> points;
	std::vector<CameraLocationData> locationData;
    PinholeCameraIntrinsics intrinsics, lockParams;
    LineDistortionEstimatorParameters distortionEstimationParams;
    LensDistortionModelParameters distortionParams;
	CameraConstraints constraints;
    bool forceZeroSkew;
};

#endif
