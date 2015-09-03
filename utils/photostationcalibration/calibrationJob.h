#ifndef CALIBRATIONJOB
#define CALIBRATIONJOB

#include <string>
#include <vector>

#include "rgb24Buffer.h"
#include "displacementBuffer.h"
#include "selectableGeometryFeatures.h"
#include "checkerboardDetectionParameters.h"
#include "lensDistortionModelParameters.h"
#include "lineDistortionEstimatorParameters.h"
#include "distortionApplicationParameters.h"
#include "chessBoardDetector.h"
#include "calibration_structs.h"
#include "photoStationCalibrator.h"

struct ImageData
{
    std::string sourceFileName;
    std::string undistortedFileName;
    corecvs::ObservationList sourcePattern;
    corecvs::ObservationList undistortedPattern;
    LocationData location;

    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(sourceFileName, std::string(""), "sourceFileName");
        visitor.visit(undistortedFileName, std::string(""), "undistortedFileName");
        visitor.visit((std::vector<PointObservation>&)sourcePattern, "sourcePattern");
        visitor.visit((std::vector<PointObservation>&)undistortedPattern, "undistortedPattern");
        visitor.visit(location, LocationData(), "viewLocation");
    }
};

struct CalibrationSetupEntry
{
    int cameraId;
    int imageId;

    template <class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(cameraId, 0, "cameraId");
        visitor.visit(imageId, 0, "imageId");
    }
};

struct CalibrationSettings
{
    ChessBoardDetectorParams chessBoardDetectorParams;
    ChessBoardAssemblerParams chessBoardAssemblerParams;
    ChessBoardCornerDetectorParams chessBoardCornerDetectorParams;

    bool useOpenCVDetector = false;
    CheckerboardDetectionParameters openCvDetectorParameters;

    LineDistortionEstimatorParameters distortionEstimationParameters;

    DistortionApplicationParameters distortionApplicationParameters;

    // TODO: move to some <<CalibratorParams>> structure
    bool singleCameraCalibratorUseZhangPresolver = true;
    bool singleCameraCalibratorUseLMSolver = true;
    CameraConstraints singleCameraCalibratorConstraints = CameraConstraints::ZERO_SKEW | CameraConstraints::EQUAL_FOCAL | CameraConstraints::LOCK_SKEW;

    bool photostationCalibratorUseBFSPresolver = true;
    bool photostationCalibratorUseLMSolver = true;
    CameraConstraints photostationCalibratorConstraints = CameraConstraints::ZERO_SKEW | CameraConstraints::EQUAL_FOCAL | CameraConstraints::LOCK_SKEW;

    CameraIntrinsics_ calibrationLockParams;

    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(chessBoardDetectorParams, ChessBoardDetectorParams(), "chessBoardDetectorParams");
        visitor.visit(chessBoardAssemblerParams, ChessBoardAssemblerParams(), "chessBoardAssemblerParams");
        visitor.visit(chessBoardCornerDetectorParams, ChessBoardCornerDetectorParams(), "chessBoardCornerDetectorParams");

        visitor.visit(useOpenCVDetector, false, "useOpenCVDetector");
        visitor.visit(openCvDetectorParameters, CheckerboardDetectionParameters(), "openCvDetectorParameters");

        visitor.visit(distortionEstimationParameters, LineDistortionEstimatorParameters(), "lineDistortionEstimationParameters");
        visitor.visit(distortionApplicationParameters, DistortionApplicationParameters(), "distortionApplicationParameters");

        visitor.visit(singleCameraCalibratorUseZhangPresolver, true, "singleCameraCalibratorUseZhangPresolver");
        visitor.visit(singleCameraCalibratorUseLMSolver, true, "singleCameraCalibratorUseLMSolver");
        auto m = asInteger(singleCameraCalibratorConstraints);
        visitor.visit(m, asInteger(CameraConstraints::ZERO_SKEW | CameraConstraints::EQUAL_FOCAL | CameraConstraints::LOCK_SKEW), "singleCameraCalibratorConstraints");
        singleCameraCalibratorConstraints = static_cast<CameraConstraints>(m);

        visitor.visit(photostationCalibratorUseBFSPresolver, true, "photostationCalibratorUseBFSPresolver");
        visitor.visit(photostationCalibratorUseLMSolver, true, "photostationCalibratorUseLMSolver");
        m = asInteger(photostationCalibratorConstraints);
        visitor.visit(m, asInteger(CameraConstraints::ZERO_SKEW | CameraConstraints::EQUAL_FOCAL | CameraConstraints::LOCK_SKEW), "photostationCalibratorConstraints");
        photostationCalibratorConstraints = static_cast<CameraConstraints>(m);

        visitor.visit(calibrationLockParams, CameraIntrinsics_(), "calibrationLockParams");
    }
};

struct CalibrationJob
{
    Photostation photostation;
    std::vector<LocationData> calibrationSetupLocations;
    std::vector<std::vector<ImageData>> observations;
    std::vector<std::vector<CalibrationSetupEntry>> calibrationSetups;

    CalibrationSettings settings;

    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(photostation, Photostation(), "photostation");
        visitor.visit(calibrationSetupLocations, "calibrationSetupLocations");
        visitor.visit(calibrationSetups, "calibrationSetups");
        visitor.visit(observations, "observations");
        visitor.visit(settings, CalibrationSettings(), "algorithmSettings");
    }

    static corecvs::RGB24Buffer LoadImage(const std::string& path);
    static void SaveImage(const std::string& path, corecvs::RGB24Buffer &buffer);
    
    bool detectChessBoard(corecvs::RGB24Buffer &buffer, corecvs::ObservationList &list);
    bool detectChessBoard(corecvs::RGB24Buffer &buffer, corecvs::SelectableGeometryFeatures &features);
    bool detectChessBoard(corecvs::RGB24Buffer &buffer, corecvs::SelectableGeometryFeatures *features = nullptr, corecvs::ObservationList *list = nullptr);
    void allDetectChessBoard(bool distorted = true);
   
    bool estimateDistortion(corecvs::SelectableGeometryFeatures &features, double w, double h, LensDistortionModelParameters &params);
    bool estimateDistortion(corecvs::ObservationList &list, double w, double h, LensDistortionModelParameters &params);
    void allEstimateDistortion();

    void prepareUndistortionTransformation(LensDistortionModelParameters &source, double w, double h, corecvs::DisplacementBuffer &dest, double &newW, double &newH);
    void removeDistortion(corecvs::RGB24Buffer &src, corecvs::RGB24Buffer &dst, LensDistortionModelParameters &params);
    void removeDistortion(corecvs::RGB24Buffer &src, corecvs::RGB24Buffer &dst, corecvs::DisplacementBuffer &transform, double outW, double outH);
    void allRemoveDistortion();

    bool calibrateSingleCamera(int cameraId);
    void allCalibrateSingleCamera();

    void calibratePhotostation();
    void calibratePhotostation(int N, int M, PhotoStationCalibrator &calibrator, std::vector<MultiCameraPatternPoints> &points, std::vector<CameraIntrinsics_> &intrinsics, std::vector<std::vector<LocationData>> &locations, bool runBFS, bool runLM);
    void calibrate();
};

#endif
