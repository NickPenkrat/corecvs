#ifndef PHOTOSTATION_CALIBRATOR_H
#define PHOTOSTATION_CALIBRATOR_H

#include <vector>

#include "calibrationPhotostation.h"
#include "lensDistortionModelParameters.h"
#include "lineDistortionEstimatorParameters.h"
#include "levenmarq.h"


//#define PENALIZE_QNORM

//#define CALIB_FACTOR

struct PhotoStationCalibrator
{
public:
    PhotoStationCalibrator(CameraConstraints constraints = CameraConstraints::NONE, const LineDistortionEstimatorParameters &distortionEstimationParameters = LineDistortionEstimatorParameters(), const double lockFactor = 1.0);

    // Add camera
    void addCamera(PinholeCameraIntrinsics &intrinsics, const LensDistortionModelParameters &params = LensDistortionModelParameters());

    // Add calibration setup and corespondences for specified cameras
    void addCalibrationSetup(std::vector<int> &cameraIds, std::vector<CameraLocationData> &cameraLocations, MultiCameraPatternPoints &points);

    // Runs solver
    void solve(bool runPresolver = true, bool runNonLinear = true, int LMiterations = 1000);
    void solve(bool runPresolver, bool runNonLinear, CameraConstraints constraints, int LMiterations = 1000);

    // Returns calibration setup poses
    std::vector<CameraLocationData> getCalibrationSetups();

    // Returns calibrated photostation
    Photostation getPhotostation();

    // Returns RMSE for reprojection
    double getRmseReprojectionError();

    // Returns full error
    void getFullReprojectionError(std::vector<double> &err);

    // Centers on LSQ point
    void recenter();

    void validate();
    double factor = 1.0;
private:
    void getFullReprojectionError(double out[]);
    int getInputNum() const;
    int getStructInputNum() const;
    int getOutputNum() const;
    int getStructOutputNum() const;

    void readParams(const double in[]);
    void readStructureParams(const double in[]);
    void writeStructureParams(double out[]);
    void writeParams(double out[]);

    // XXX: A long time ago in a galaxy far, far away... I've thought that this
    //      stuff works slowly because of slow cost-function computation.
    //      But later I discoverd that our LM implementation (and "BLAS"
    //      subsystem) are very slow themselves... To be continued
    struct ParallelErr
    {
        ParallelErr(PhotoStationCalibrator *calibrator, std::vector<int> *offset, double *out) : calibrator(calibrator), offset(offset), out(out) {}
        void operator() (const corecvs::BlockedRange<int> &r) const
        {
            auto& absoluteSetupLocation = calibrator->absoluteSetupLocation;
            auto& patternPoints = calibrator->patternPoints;
            auto& relativeCameraPositions = calibrator->relativeCameraPositions;
            int N = calibrator->N;
            bool shouldDistort = !!(calibrator->constraints & CameraConstraints::UNLOCK_DISTORTION);

            for (int j = r.begin(); j != r.end(); ++j)
            {
                int idx = (*offset)[j];
                Quaternion& Qs = absoluteSetupLocation[j].orientation;
                Vector3dd & Cs = absoluteSetupLocation[j].position;

                for (int i = 0; i < N; ++i)
                {
                    if (patternPoints[j][i].empty())
                        continue;

                    for (PointObservation& pt: patternPoints[j][i])
                    {
                        Vector3dd p = pt.point;

                        p[1] *= calibrator->factor;

                        Vector2dd res = relativeCameraPositions[i].project(Qs * (p - Cs));
                        if (shouldDistort) {
                            res = relativeCameraPositions[i].distortion.mapForward(res);
                        }

                        Vector2dd diff = res - pt.projection;

                        out[idx++] = diff.x();
                        out[idx++] = diff.y();
                    }
                }
            }
        }

        PhotoStationCalibrator* calibrator;
        std::vector<int>* offset;
        double *out;
    };

    struct LMCostFunction : public corecvs::FunctionArgs
    {
        LMCostFunction(PhotoStationCalibrator *calibrator)
            : FunctionArgs(calibrator->getInputNum(), calibrator->getOutputNum()), calibrator(calibrator)
        {
        }
        void operator()(const double in[], double out[]);
        PhotoStationCalibrator *calibrator;
    };

    struct LMCostFunctionNormalizer : public corecvs::FunctionArgs
    {
        LMCostFunctionNormalizer(PhotoStationCalibrator *calibrator)
            : FunctionArgs(calibrator->getInputNum(), calibrator->getInputNum()), calibrator(calibrator)
        {
        }
        void operator()(const double in[], double out[]);
        PhotoStationCalibrator *calibrator;
    };

    struct LMStructure : public corecvs::FunctionArgs
    {
        LMStructure(PhotoStationCalibrator *calibrator)
            : FunctionArgs(calibrator->getStructInputNum(), calibrator->getStructOutputNum()), calibrator(calibrator)
        {
        }
        void operator() (const double in[], double out[]);

        PhotoStationCalibrator *calibrator;
    };

    struct LMStructureNormalizer : public corecvs::FunctionArgs
    {
        LMStructureNormalizer(PhotoStationCalibrator *calibrator)
            : FunctionArgs(calibrator->getStructInputNum(), calibrator->getStructInputNum()), calibrator(calibrator)
        {
        }

        void operator() (const double in[], double out[]);
        PhotoStationCalibrator *calibrator;
    };
    // LM-solver
    void refineGuess(int LMiterations);
    void refineStruct();


    // Prepares initialization for non-linear search
    void solveInitialLocations();
    bool getMSTSolveOrder(std::vector<std::pair<int, int>> &order);
    // Solves calibration setup pose from absolute and relative camera pose
    void solveCameraToSetup(const CameraLocationData &realLocation, int camera, int setup);
    // Solves relative camera pose from calibration setup pose and absolute camera pose
    void solveSetupToCamera(const CameraLocationData &realLocation, int camera, int setup);

    int N, M, K, L;

    std::vector<CameraModel> relativeCameraPositions;
    std::vector<CameraLocationData> absoluteSetupLocation;

    std::vector<std::vector<std::pair<bool, CameraLocationData> > > initialGuess;
    std::vector<MultiCameraPatternPoints> patternPoints;

    CameraConstraints constraints;
    LineDistortionEstimatorParameters distortionEstimationParams;
};


#endif
