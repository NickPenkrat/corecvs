#ifndef CALIBRATION_JOB_H_
#define CALIBRATION_JOB_H_

#include <string>
#include <vector>
#include <atomic>

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
#include "statusTracker.h"

#ifdef  LoadImage
# undef LoadImage
#endif

typedef std::array<corecvs::Vector2dd, 2> Rect;
using std::string;
using corecvs::ObservationList;

struct ImageData
{
    string              sourceFileName;
    string              undistortedFileName;
    ObservationList     sourcePattern;
    ObservationList     undistortedPattern;
    CameraLocationData  location;

    double              distortionRmse;
    double              distortionMaxError;
    double              calibrationRmse;
    double              calibrationMaxError;
    double              singleCameraRmse;
    double              singleCameraMaxError;
    double              fullCameraRmse;
    double              fullCameraMaxError;

    ImageData()
    {
        DefaultSetter setter;
        accept(setter);
    }

    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(sourceFileName,       std::string(""), "sourceFileName");
        visitor.visit(undistortedFileName,  std::string(""), "undistortedFileName");
        visitor.visit((std::vector<PointObservation>&)sourcePattern,      "sourcePattern");
        visitor.visit((std::vector<PointObservation>&)undistortedPattern, "undistortedPattern");
        visitor.visit(location,             CameraLocationData(),         "viewLocation");
        visitor.visit(distortionRmse,       -1.0, "distortionRmse");
        visitor.visit(distortionMaxError,   -1.0, "distortionMaxError");
        visitor.visit(calibrationRmse,      -1.0, "calibrationRmse");
        visitor.visit(calibrationMaxError,  -1.0, "calibrationMaxError");
        visitor.visit(calibrationRmse,      -1.0, "singleCameraRmse");
        visitor.visit(calibrationMaxError,  -1.0, "singleCameraMaxError");
        visitor.visit(fullCameraRmse,       -1.0, "fullCameraRmse");
        visitor.visit(fullCameraMaxError,   -1.0, "fullCameraMaxError");
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
        visitor.visit(imageId , 0, "imageId" );
    }
};

struct CalibrationSettings
{
    /* TODO : rename this */
    ChessBoardAssemblerParams           chessBoardAssemblerParams;
    ChessBoardCornerDetectorParams      chessBoardCornerDetectorParams;
    BoardAlignerParams                  boardAlignerParams;
    CheckerboardDetectionParameters     openCvDetectorParameters;

    LineDistortionEstimatorParameters   distortionEstimationParameters;
    DistortionApplicationParameters     distortionApplicationParameters;

    PinholeCameraIntrinsics             calibrationLockParams;

    double                              forceFactor = 1.0;

    // TODO: move to some <<CalibratorParams>> structure
    bool singleCameraCalibratorUseZhangPresolver = true;
    bool singleCameraCalibratorUseLMSolver       = true;
    CameraConstraints singleCameraCalibratorConstraints = CameraConstraints::ZERO_SKEW | CameraConstraints::EQUAL_FOCAL | CameraConstraints::LOCK_SKEW;
    int  singleCameraLMiterations                = 1000;

    bool photostationCalibratorUseBFSPresolver   = true;
    bool photostationCalibratorUseLMSolver       = true;
    CameraConstraints photostationCalibratorConstraints = CameraConstraints::ZERO_SKEW | CameraConstraints::EQUAL_FOCAL | CameraConstraints::LOCK_SKEW;
    int  photostationLMiterations                = 1000;

    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
//        visitor.visit(chessBoardDetectorParams, ChessBoardDetectorParams(), "chessBoardDetectorParams");
        visitor.visit(chessBoardAssemblerParams,                ChessBoardAssemblerParams(), "chessBoardAssemblerParams");
        visitor.visit(chessBoardCornerDetectorParams,           ChessBoardCornerDetectorParams(), "chessBoardCornerDetectorParams");
        visitor.visit(boardAlignerParams,                       BoardAlignerParams(), "boardAlignerParams");

//        visitor.visit(useOpenCVDetector, false, "useOpenCVDetector");
        visitor.visit(forceFactor,                              1.0, "forceYFactor");

        visitor.visit(openCvDetectorParameters,                 CheckerboardDetectionParameters(), "openCvDetectorParameters");

        visitor.visit(distortionEstimationParameters,           LineDistortionEstimatorParameters(), "lineDistortionEstimationParameters");
        visitor.visit(distortionApplicationParameters,          DistortionApplicationParameters(), "distortionApplicationParameters");

        visitor.visit(singleCameraCalibratorUseZhangPresolver,  true, "singleCameraCalibratorUseZhangPresolver");
        visitor.visit(singleCameraCalibratorUseLMSolver,        true, "singleCameraCalibratorUseLMSolver");

        auto m = asInteger(singleCameraCalibratorConstraints);
        visitor.visit(m, asInteger(CameraConstraints::ZERO_SKEW | CameraConstraints::EQUAL_FOCAL | CameraConstraints::LOCK_SKEW), "singleCameraCalibratorConstraints");
        singleCameraCalibratorConstraints = static_cast<CameraConstraints>(m);

        visitor.visit(photostationCalibratorUseBFSPresolver,    true, "photostationCalibratorUseBFSPresolver");
        visitor.visit(photostationCalibratorUseLMSolver,        true, "photostationCalibratorUseLMSolver");

        m = asInteger(photostationCalibratorConstraints);
        visitor.visit(m, asInteger(CameraConstraints::ZERO_SKEW | CameraConstraints::EQUAL_FOCAL | CameraConstraints::LOCK_SKEW), "photostationCalibratorConstraints");
        photostationCalibratorConstraints = static_cast<CameraConstraints>(m);

        visitor.visit(calibrationLockParams,                    PinholeCameraIntrinsics(), "calibrationLockParams");
        visitor.visit(singleCameraLMiterations,                 1000, "singleCameraLMiterations");
        visitor.visit(photostationLMiterations,                 1000, "photostationLMiterations");
    }
};

struct CalibrationJob
{
    Photostation                                    photostation;
    std::vector<CameraLocationData>                 calibrationSetupLocations;
    std::vector<std::vector<ImageData>>             observations;
    std::vector<std::vector<CalibrationSetupEntry>> calibrationSetups;
    double                                          totalFullErrorMax           = -1.0,
                                                    totalFullErrorRMSE          = -1.0,
                                                    totalCalibrationErrorMax    = -1.0,
                                                    totalCalibrationErrorRMSE   = -1.0,
                                                    totalReconstructionErrorMax = -1.0,
                                                    totalReconstructionErrorRMSE= -1.0;

    bool                                            calibrated = false;

    CalibrationSettings                             settings;
    // TODO: Should we serialize it?!
    StatusTracker*                                  state = nullptr;

    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(photostation,                 Photostation(),         "photostation");
        visitor.visit(calibrationSetupLocations,                            "calibrationSetupLocations");
        visitor.visit(calibrationSetups,                                    "calibrationSetups");
        visitor.visit(observations,                                         "observations");
        visitor.visit(settings,                     CalibrationSettings(),  "algorithmSettings");
        visitor.visit(calibrated,                   false,                  "calibrated");
        visitor.visit(totalFullErrorMax,            -1.0,                   "totalFullErrorMax");
        visitor.visit(totalFullErrorRMSE,           -1.0,                   "totalFullErrorRMSE");
        visitor.visit(totalCalibrationErrorMax,     -1.0,                   "totalCalibrationErrorMax");
        visitor.visit(totalCalibrationErrorRMSE,    -1.0,                   "totalCalibrationErrorRMSE");
        visitor.visit(totalReconstructionErrorMax,  -1.0,                   "totalReconstructionErrorMax");
        visitor.visit(totalReconstructionErrorRMSE, -1.0,                   "totalReconstructionErrorRMSE");
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

    void    prepareUndistortionTransformation(int camId, corecvs::DisplacementBuffer &dest);
    void    removeDistortion(corecvs::RGB24Buffer &src, corecvs::RGB24Buffer &dst, corecvs::DisplacementBuffer &transform, double outW, double outH);
    void    allRemoveDistortion();

    bool    calibrateSingleCamera(int cameraId);
    void    allCalibrateSingleCamera();

    void    computeSingleCameraErrors();
    void    computeCalibrationErrors();
    void    computeFullErrors();
    void    computeReconstructionError();
    void    calibratePhotostation();
    void    calibratePhotostation(int N, int M, PhotoStationCalibrator &calibrator, std::vector<MultiCameraPatternPoints> &points, std::vector<PinholeCameraIntrinsics> &intrinsics, std::vector<LensDistortionModelParameters> &distortions, std::vector<std::vector<CameraLocationData>> &locations, bool runBFS, bool runLM);
    void    calibrate();

    ///
    /// \brief fit
    /// Repositioning camera into E-N plane by first 6 cameras
    /// \param referenceLayerCamerasCount
    ///
    void    fit(int referenceLayerCamerasCount);

    void    calculateRedundancy(std::vector<int> &cameraImagesCount, std::vector<std::vector<int>> &cameraCameraRelationships, std::vector<int> &redundantSingleCamera, int &redundancyPhotostation);

    void    reorient(const std::vector<int> &topLayerIdx);
    void    reorient(const corecvs::Vector3dd T, const corecvs::Quaternion Q);

    double  factor = 1.0;
    std::vector<double> factors;

public:
    corecvs::Statistics stats;
};

#endif // CALIBRATION_JOB_H_
