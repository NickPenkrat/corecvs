#ifndef PHOTOSTATIONCALIBRATOR
#define PHOTOSTATIONCALIBRATOR

#include <vector>

#include "calibrationStructs.h"
#include "levenmarq.h"

//#define PENALIZE_QNORM

//#define CALIB_FACTOR

struct PhotoStationCalibrator
{
public:
    PhotoStationCalibrator(CameraConstraints constraints = CameraConstraints::NONE, const double lockFactor = 1.0);

    // Add camera
    void addCamera(CameraIntrinsics &intrinsics);

    // Add calibration setup and corespondences for specified cameras
    void addCalibrationSetup(std::vector<int> &cameraIds, std::vector<LocationData> &cameraLocations, MultiCameraPatternPoints &points);

    // Runs solver
    void solve(bool runPresolver = true, bool runNonLinear = true);
    void solve(bool runPresolver, bool runNonLinear, CameraConstraints constraints);

    // Returns calibration setup poses
    std::vector<LocationData> getCalibrationSetups();

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
    int getOutputNum() const;

    void readParams(const double in[]);
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

            for (int j = r.begin(); j != r.end(); ++j)
            {
                int idx = (*offset)[j];
                auto& Qs = absoluteSetupLocation[j].orientation;
                auto& Cs = absoluteSetupLocation[j].position;
                for (int i = 0; i < N; ++i)
                {
                    if (!patternPoints[j][i].size())
                        continue;

                    for (auto& pt: patternPoints[j][i])
                    {
                        auto p = pt.second;

                        p[1] *= calibrator->factor;

                        auto diff = relativeCameraPositions[i].project(Qs * (p - Cs)) - pt.first;
                        out[idx++] = diff[0];
                        out[idx++] = diff[1];
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

    struct LMStructure : public corecvs::FunctionArgs
    {
        LMStructure(PhotoStationCalibrator *calibrator)
            : FunctionArgs(calibrator->M * 7, calibrator->L * 3), calibrator(calibrator)
        {
        }
        void operator() (const double in[], double out[]);

        PhotoStationCalibrator *calibrator;
    };
    // LM-solver
    void refineGuess();
    void refineStruct();


    // Prepares initialization for non-linear search
    double trySolveInitialLocations(std::vector<int> &order);
    void solveInitialLocations();
    // Solves calibration setup pose from absolute and relative camera pose
    void solveCameraToSetup(const LocationData &realLocation, int camera, int setup);
    // Solves relative camera pose from calibration setup pose and absolute camera pose
    void solveSetupToCamera(const LocationData &realLocation, int camera, int setup);

    int N, M, K, L;

    std::vector<Camera_> relativeCameraPositions;
    std::vector<LocationData> absoluteSetupLocation;

    std::vector<std::vector<std::pair<bool, LocationData> > > initialGuess;
    std::vector<MultiCameraPatternPoints> patternPoints;

    CameraConstraints constraints;
};


#endif
