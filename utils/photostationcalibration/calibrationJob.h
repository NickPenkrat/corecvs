#ifndef CALIBRATION_JOB_H_
#define CALIBRATION_JOB_H_

#include <string>
#include <vector>

#include "calculationStats.h"
#include "rgb24Buffer.h"
#include "displacementBuffer.h"
#include "selectableGeometryFeatures.h"
#include "checkerboardDetectionParameters.h"
#include "lensDistortionModelParameters.h"
#include "lineDistortionEstimatorParameters.h"
#include "distortionApplicationParameters.h"
#include "chessBoardDetector.h"
#include "calibrationPhotostation.h"
#include "photoStationCalibrator.h"

#ifdef  LoadImage
# undef LoadImage
#endif

struct ImageData
{
    std::string              sourceFileName;
    std::string              undistortedFileName;
    corecvs::ObservationList sourcePattern;
    corecvs::ObservationList undistortedPattern;
    CameraLocationData       location;

    double distortionRmse       = -1.0
         , distortionMaxError   = -1.0
         , calibrationRmse      = -1.0
         , calibrationMaxError  = -1.0
         , singleCameraRmse     = -1.0
         , singleCameraMaxError = -1.0;

    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(sourceFileName, std::string(""), "sourceFileName");
        visitor.visit(undistortedFileName, std::string(""), "undistortedFileName");
        visitor.visit((std::vector<PointObservation>&)sourcePattern, "sourcePattern");
        visitor.visit((std::vector<PointObservation>&)undistortedPattern, "undistortedPattern");
        visitor.visit(location, CameraLocationData(), "viewLocation");
        visitor.visit(distortionRmse, -1.0, "distortionRmse");
        visitor.visit(distortionMaxError, -1.0, "distortionMaxError");
        visitor.visit(calibrationRmse, -1.0, "calibrationRmse");
        visitor.visit(calibrationMaxError, -1.0, "calibrationMaxError");
        visitor.visit(calibrationRmse, -1.0, "singleCameraRmse");
        visitor.visit(calibrationMaxError, -1.0, "singleCameraMaxError");
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
    /* TODO : rename this */
    CheckerboardDetectionParameters     openCvDetectorParameters;

    BoardAlignerParams                  boardAlignerParams = BoardAlignerParams::GetOldBoard();

    ChessBoardAssemblerParams           chessBoardAssemblerParams;
    ChessBoardCornerDetectorParams      chessBoardCornerDetectorParams;

    double forceFactor = 1.0;

    LineDistortionEstimatorParameters   distortionEstimationParameters;

    DistortionApplicationParameters     distortionApplicationParameters;

    // TODO: move to some <<CalibratorParams>> structure
    bool singleCameraCalibratorUseZhangPresolver = true;
    bool singleCameraCalibratorUseLMSolver       = true;
    CameraConstraints singleCameraCalibratorConstraints = CameraConstraints::ZERO_SKEW | CameraConstraints::EQUAL_FOCAL | CameraConstraints::LOCK_SKEW;
    int  singleCameraLMiterations                = 1000;

    bool photostationCalibratorUseBFSPresolver   = true;
    bool photostationCalibratorUseLMSolver       = true;
    CameraConstraints photostationCalibratorConstraints = CameraConstraints::ZERO_SKEW | CameraConstraints::EQUAL_FOCAL | CameraConstraints::LOCK_SKEW;
    int  photostationLMiterations                = 1000;

    PinholeCameraIntrinsics             calibrationLockParams;

    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
//        visitor.visit(chessBoardDetectorParams, ChessBoardDetectorParams(), "chessBoardDetectorParams");
        visitor.visit(chessBoardAssemblerParams, ChessBoardAssemblerParams(), "chessBoardAssemblerParams");
        visitor.visit(chessBoardCornerDetectorParams, ChessBoardCornerDetectorParams(), "chessBoardCornerDetectorParams");
        visitor.visit(boardAlignerParams, BoardAlignerParams::GetOldBoard(), "boardAlignerParams");

//        visitor.visit(useOpenCVDetector, false, "useOpenCVDetector");
        visitor.visit(forceFactor, 1.0, "forceYFactor");

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

        visitor.visit(calibrationLockParams, PinholeCameraIntrinsics(), "calibrationLockParams");
        visitor.visit(singleCameraLMiterations, 1000, "singleCameraLMiterations");
        visitor.visit(photostationLMiterations, 1000, "photostationLMiterations");
    }
};

struct CalibrationJob
{
    Photostation                                    photostation;
    std::vector<CameraLocationData>                 calibrationSetupLocations;
    std::vector<std::vector<ImageData>>             observations;
    std::vector<std::vector<CalibrationSetupEntry>> calibrationSetups;

    bool                                            calibrated = false;

    CalibrationSettings                             settings;

    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(photostation, Photostation(), "photostation");
        visitor.visit(calibrationSetupLocations, "calibrationSetupLocations");
        visitor.visit(calibrationSetups, "calibrationSetups");
        visitor.visit(observations, "observations");
        visitor.visit(settings, CalibrationSettings(), "algorithmSettings");
        visitor.visit(calibrated, false, "calibrated");
    }

    static corecvs::RGB24Buffer LoadImage(const std::string& path);
    static void                 SaveImage(const std::string& path, corecvs::RGB24Buffer &buffer);
    
    bool    detectChessBoard(corecvs::RGB24Buffer &buffer, corecvs::ObservationList &list);
    bool    detectChessBoard(corecvs::RGB24Buffer &buffer, corecvs::SelectableGeometryFeatures &features);
    bool    detectChessBoard(corecvs::RGB24Buffer &buffer, corecvs::SelectableGeometryFeatures *features = nullptr, corecvs::ObservationList *list = nullptr);
    void    allDetectChessBoard(bool distorted = true);
   
    bool    estimateDistortion(corecvs::SelectableGeometryFeatures &features, double w, double h, LensDistortionModelParameters &params);
    bool    estimateDistortion(corecvs::ObservationList &list, double w, double h, LensDistortionModelParameters &params);
    void    computeDistortionError(corecvs::ObservationList &list, LensDistortionModelParameters &params, double &rmse, double &maxError);
    void    computeDistortionError(corecvs::SelectableGeometryFeatures &sgf, LensDistortionModelParameters &params, double &rmse, double &maxError);
    void    allEstimateDistortion();

    void    prepareUndistortionTransformation(LensDistortionModelParameters &source, double w, double h, corecvs::DisplacementBuffer &dest, double &newW, double &newH);
    void    removeDistortion(corecvs::RGB24Buffer &src, corecvs::RGB24Buffer &dst, LensDistortionModelParameters &params);
    void    removeDistortion(corecvs::RGB24Buffer &src, corecvs::RGB24Buffer &dst, corecvs::DisplacementBuffer &transform, double outW, double outH);
    void    allRemoveDistortion();

    bool    calibrateSingleCamera(int cameraId);
    void    allCalibrateSingleCamera();

    void    computeSingleCameraErrors();
    void    computeCalibrationErrors();
    void    calibratePhotostation();
    void    calibratePhotostation(int N, int M, PhotoStationCalibrator &calibrator, std::vector<MultiCameraPatternPoints> &points, std::vector<PinholeCameraIntrinsics> &intrinsics, std::vector<std::vector<CameraLocationData>> &locations, bool runBFS, bool runLM);
    void    calibrate();

    void    calculateRedundancy(std::vector<int> &cameraImagesCount, std::vector<std::vector<int>> &cameraCameraRelationships, std::vector<int> &redundantSingleCamera, int &redundancyPhotostation);

    double  factor = 1.0;
    std::vector<double> factors;

public:
    corecvs::Statistics stats;
};

#endif // CALIBRATION_JOB_H_
