#include "calibrationJob.h"

#include <array>

#include "openCvCheckerboardDetector.h"
#include "lmDistortionSolver.h"
#include "flatPatternCalibrator.h"
#include "photoStationCalibrator.h"

#include "qtFileLoader.h"

#include "tbbWrapper.h"

bool CalibrationJob::detectChessBoard(corecvs::RGB24Buffer &buffer, corecvs::ObservationList &list)
{
    return detectChessBoard(buffer, nullptr, &list);
}

bool CalibrationJob::detectChessBoard(corecvs::RGB24Buffer &buffer, corecvs::SelectableGeometryFeatures &features)
{
    return detectChessBoard(buffer, &features);
}

bool CalibrationJob::detectChessBoard(corecvs::RGB24Buffer &buffer, corecvs::SelectableGeometryFeatures *features, corecvs::ObservationList *list)
{
    PatternDetector *patternDetector;
    if (settings.openCvDetectorParameters.algorithm() == CheckerboardDetectionAlgorithm::OPENCV_DETECTOR)
    {
        patternDetector = new OpenCvCheckerboardDetector(settings.openCvDetectorParameters, settings.boardAlignerParams);
        G8Buffer* channel = buffer.getChannel(settings.openCvDetectorParameters.channel());
        patternDetector->detectPattern(*channel);
        delete channel;
    }
    else
    {
        patternDetector = new ChessboardDetector(settings.openCvDetectorParameters, settings.boardAlignerParams, settings.chessBoardCornerDetectorParams, settings.chessBoardAssemblerParams);
        patternDetector->setStatistics(&stats);
        patternDetector->detectPattern(buffer);

    }

    if (features)
        patternDetector->getPointData(*features);
    if (list)
        patternDetector->getPointData(*list);

    delete patternDetector;
    return (features && features->mPoints.size()) || (list && list->size());
}

void CalibrationJob::allDetectChessBoard(bool distorted)
{
    int N = (int)observations.size();
    photostation.cameras.resize(N);
    auto psIterator = photostation.cameras.begin();

    for (auto& c: observations)
    {
        for (auto& v: c)
        {
            std::string filename = distorted ? v.sourceFileName : v.undistortedFileName;
            corecvs::RGB24Buffer buffer = LoadImage(filename);
            if (buffer.h && buffer.w)
            {
                (distorted ? psIterator->intrinsics.distortedSize : psIterator->intrinsics.size) = corecvs::Vector2dd(buffer.w, buffer.h);
            }
            detectChessBoard(buffer, distorted ? v.sourcePattern : v.undistortedPattern);
        }
        ++psIterator;
    }
}

void CalibrationJob::computeDistortionError(corecvs::ObservationList &list, LensDistortionModelParameters &params, double &rmse, double &maxError)
{
    corecvs::SelectableGeometryFeatures sgf;
    sgf.addAllLinesFromObservationList(list);
    computeDistortionError(sgf, params, rmse, maxError);
}

void CalibrationJob::computeDistortionError(corecvs::SelectableGeometryFeatures &sgf, LensDistortionModelParameters &params, double &rmse, double &maxError)
{
    corecvs::RadialCorrection corrector(params);
    corecvs::LMLinesDistortionSolver solver;
    solver.lineList = &sgf;
    solver.computeCosts(corrector, true);

    EllipticalApproximation1d &cost = solver.costs[LineDistortionEstimatorCost::LINE_DEVIATION_COST];
    rmse = cost.getRadiusAround0();
    maxError = cost.getMax();
}

bool CalibrationJob::estimateDistortion(corecvs::ObservationList &list, double w, double h, LensDistortionModelParameters &params)
{
    corecvs::SelectableGeometryFeatures sgf;
    sgf.addAllLinesFromObservationList(list);
    return estimateDistortion(sgf, w, h, params);
}

bool CalibrationJob::estimateDistortion(corecvs::SelectableGeometryFeatures &features, double w, double h, LensDistortionModelParameters &params)
{
    if (!features.mPoints.size())
        return false;

    corecvs::LMLinesDistortionSolver solver;
    solver.initialCenter = corecvs::Vector2dd(w / 2.0, h / 2.0);
    solver.lineList = &features;
    solver.parameters = settings.distortionEstimationParameters;

    corecvs::RadialCorrection correction = solver.solve();
    params = correction.mParams;

    // TODO: check if we can detect a failure of distortion estimation
    return true;
}

struct ParallelDistortionEstimator
{
    void operator() (const corecvs::BlockedRange<int> &r) const
    {
        for (int cam = r.begin(); cam < r.end(); ++cam)
        {
            corecvs::SelectableGeometryFeatures sgf;
            for (auto& v: job->observations[cam])
            {
                sgf.addAllLinesFromObservationList(v.sourcePattern);
            }
            auto& psIterator = job->photostation.cameras[cam];
            job->estimateDistortion(sgf, psIterator.intrinsics.distortedSize[0], psIterator.intrinsics.distortedSize[1], psIterator.distortion);
            for (auto& v: job->observations[cam])
            {
                job->computeDistortionError(v.sourcePattern, psIterator.distortion, v.distortionRmse, v.distortionMaxError);
            }
        }
    }

    ParallelDistortionEstimator(CalibrationJob *job) : job(job)
    {}

    CalibrationJob *job;
};

void CalibrationJob::allEstimateDistortion()
{
    corecvs::parallelable_for(0, (int)photostation.cameras.size(), ParallelDistortionEstimator(this));
}

typedef std::array<corecvs::Vector2dd, 2> Rect;

void CalibrationJob::prepareUndistortionTransformation(LensDistortionModelParameters &source, double w, double h, corecvs::DisplacementBuffer &result, double &newW, double &newH)
{
    RadialCorrection correction(source);
    auto undistParams = settings.distortionApplicationParameters;
    if (settings.distortionApplicationParameters.forceScale())
    {
        correction.mParams.setScale(source.scale());
    }

    Rect input = { corecvs::Vector2dd(0.0, 0.0), corecvs::Vector2dd(w, h) };
    Rect outCir, outIns;

    correction.getCircumscribedImageRect(input[0][0], input[0][1], input[1][0], input[1][1], outCir[0], outCir[1]);
    correction.getInscribedImageRect    (input[0][0], input[0][1], input[1][0], input[1][1], outIns[0], outIns[1]);

    Rect output = input;
    corecvs::Vector2dd shift(0.0);

    switch (undistParams.resizePolicy())
    {
        case DistortionResizePolicy::FORCE_SIZE:
            output = { corecvs::Vector2dd(0.0, 0.0), corecvs::Vector2dd(undistParams.newW(), undistParams.newH()) };
            shift[0] = correction.center()[0] / w * output[1][0];
            shift[1] = correction.center()[1] / h * output[1][1];
            break;
        case DistortionResizePolicy::TO_FIT_RESULT:
            output = outCir;
            shift = -output[0];
            break;
        case DistortionResizePolicy::TO_NO_GAPS:
            output = outIns;
            shift = -output[0];
        case DistortionResizePolicy::NO_CHANGE:
        default:
            break;
    }

    auto wh = output[1] - output[0];
    newW = wh[0];
    newH = wh[1];

    if (undistParams.adoptScale())
    {
        double aspect = std::max(newW / w, newH / h);
        correction.mParams.setScale(1.0 / aspect);
    }
    else
    {
        correction.addShiftX = shift[0];
        correction.addShiftY = shift[1];
    }

    auto* foo = DisplacementBuffer::CacheInverse(
            &correction, newH, newW,
            output[0][0], output[0][1],
            output[1][0], output[1][1],
            0.25, 0.0);
    result = *foo;
    delete foo;
}

void CalibrationJob::removeDistortion(corecvs::RGB24Buffer &src, corecvs::RGB24Buffer &dst, LensDistortionModelParameters &params)
{
    corecvs::DisplacementBuffer transform;
    double newW, newH;
    prepareUndistortionTransformation(params, src.w, src.h, transform, newW, newH);

    removeDistortion(src, dst, transform, newW, newH);
}

void CalibrationJob::removeDistortion(corecvs::RGB24Buffer &src, corecvs::RGB24Buffer &dst, corecvs::DisplacementBuffer &transform, double outW, double outH)
{
    auto* res = src.doReverseDeformationBlTyped<corecvs::DisplacementBuffer>(&transform, outH, outW);
    dst = *res;
    delete res;
}

struct ParallelDistortionRemoval
{
    void operator() (const corecvs::BlockedRange<int> &r) const
    {
        for (int camId = r.begin(); camId < r.end(); ++camId)
        {
            auto& observationsIterator = job->observations[camId];
            auto& cam = job->photostation.cameras[camId];

            corecvs::DisplacementBuffer transform;
            double newW, newH;
            job->prepareUndistortionTransformation(cam.distortion, cam.intrinsics.distortedSize[0], cam.intrinsics.distortedSize[1], transform, newW, newH);
            cam.intrinsics.size[0] = newW;
            cam.intrinsics.size[1] = newH;
            for (auto& ob: observationsIterator)
            {
                corecvs::RGB24Buffer source = job->LoadImage(ob.sourceFileName), dst;
                job->removeDistortion(source, dst, transform, newW, newH);
                job->SaveImage(ob.undistortedFileName, dst);
            }
        }
    }

    ParallelDistortionRemoval(CalibrationJob *job) : job(job)
    {}

    CalibrationJob *job;
};

void CalibrationJob::allRemoveDistortion()
{
    corecvs::parallelable_for (0, (int)photostation.cameras.size(), ParallelDistortionRemoval(this));
}

void CalibrationJob::SaveImage(const std::string &path, corecvs::RGB24Buffer &img)
{
    QTFileLoader().save(path, &img, 100);
}

corecvs::RGB24Buffer CalibrationJob::LoadImage(const std::string &path)
{
    auto* buffer = QTRGB24Loader().load(path);
    if (!buffer)
        return corecvs::RGB24Buffer();
    corecvs::RGB24Buffer res = *buffer;
    return res;
}

bool CalibrationJob::calibrateSingleCamera(int cameraId)
{
    std::vector<LocationData> locations;
    int valid_locations = 0;

    FlatPatternCalibrator calibrator(settings.singleCameraCalibratorConstraints, settings.calibrationLockParams);

    for (auto& o: observations[cameraId])
    {
        PatternPoints3d patternPoints;
        for (auto& p: o.undistortedPattern)
            patternPoints.emplace_back(p.projection, p.point);

        if (patternPoints.size())
        {
            calibrator.addPattern(patternPoints);
            valid_locations++;
        }
    }

    if (valid_locations > 2)
    {
        calibrator.factor = settings.forceFactor;
        valid_locations = 0;
        calibrator.solve(settings.singleCameraCalibratorUseZhangPresolver, settings.singleCameraCalibratorUseLMSolver);

        photostation.cameras[cameraId].intrinsics = calibrator.getIntrinsics();
        locations = calibrator.getExtrinsics();

        for (auto& o: observations[cameraId])
        {
            if (o.undistortedPattern.size())
                o.location = locations[valid_locations++];
        }
        factors[cameraId] = calibrator.factor;
        return true;
    }
    return false;
}

struct ParallelSingleCalibrator
{
    void operator() (const corecvs::BlockedRange<int> &r) const
    {
        for (int cameraId = r.begin(); cameraId < r.end(); ++cameraId)
            job->calibrateSingleCamera(cameraId);
    }

    CalibrationJob *job;

    ParallelSingleCalibrator(CalibrationJob *job) : job(job) {}
};

void CalibrationJob::allCalibrateSingleCamera()
{
    factors.resize(photostation.cameras.size(), 1.0);
    corecvs::parallelable_for(0, (int)photostation.cameras.size(), ParallelSingleCalibrator(this));
    double fac = 0.0;
    for (auto& f: factors)
        fac += f;
    fac /= factors.size();
    factor = fac;
    std::cout << "OPTFAC_MEAN: " << factor << std::endl;
}

void CalibrationJob::calibratePhotostation(int N, int /*M*/, PhotoStationCalibrator &calibrator, std::vector<MultiCameraPatternPoints> &points, std::vector<PinholeCameraIntrinsics> &intrinsics, std::vector<std::vector<LocationData>> &locations, bool runBFS, bool runLM)
{
    for (auto& ci: intrinsics)
    {
        calibrator.addCamera(ci);
    }
    calibrator.factor = factor;
	std::vector<int> cnt(N);
	int set = 0;
    for (auto& setup: points)
	{
		MultiCameraPatternPoints pts;
		std::vector<int> active;
		std::vector<LocationData> locs;
		for (int i = 0; i < N; ++i)
		{
			if (setup[i].size())
			{
				active.push_back(i);
				pts.push_back(setup[i]);
				locs.push_back(locations[set][i]);
			}
		}
		calibrator.addCalibrationSetup(active, locs, pts);
		set++;
	}
	if (runBFS)
        calibrator.solve(true, false);
    calibrator.recenter(); // Let us hope it'll speedup...
    if (runLM)
        calibrator.solve(false, true);
    calibrator.recenter();
    factor = calibrator.factor;
}

void CalibrationJob::calibratePhotostation()
{
    int M = (int)calibrationSetups.size();
    int N = (int)photostation.cameras.size();
    std::vector<MultiCameraPatternPoints> points(M);
    for (int i = 0; i < M; ++i)
    {
        points[i].resize(N);
        for (auto& s: calibrationSetups[i])
        {
            points[i][s.cameraId].clear();
            for (auto& p: observations[s.cameraId][s.imageId].undistortedPattern)
                points[i][s.cameraId].emplace_back(p.projection, p.point);
        }
    }

    std::vector<PinholeCameraIntrinsics> intrinsics;
    for (auto& c: photostation.cameras)
        intrinsics.push_back(c.intrinsics);
    std::vector<std::vector<LocationData>> locations(M);
    for (int i = 0; i < M; ++i)
    {
        locations[i].resize(N);
        for (auto &o: calibrationSetups[i])
        {
            locations[i][o.cameraId] = observations[o.cameraId][o.imageId].location;
        }
    }

    PhotoStationCalibrator calibrator(settings.photostationCalibratorConstraints);
    calibratePhotostation(N, M, calibrator, points, intrinsics, locations, settings.photostationCalibratorUseBFSPresolver, settings.photostationCalibratorUseLMSolver);
    auto ps = calibrator.getPhotostation();
    for (int i = 0; i < N; ++i)
    {
        photostation.cameras[i].intrinsics = ps.cameras[i].intrinsics;
        photostation.cameras[i].extrinsics = ps.cameras[i].extrinsics;
    }

    calibrationSetupLocations = calibrator.getCalibrationSetups();
    calibrated = true;
}

void CalibrationJob::computeCalibrationErrors()
{
    auto setupLocsIterator = calibrationSetupLocations.begin();
    for (auto& s: calibrationSetups)
    {
        auto loc = *setupLocsIterator;
        photostation.location = loc;
        for (auto& v: s)
        {
            auto& view = observations[v.cameraId][v.imageId];
            int cam = v.cameraId;

            if (view.undistortedPattern.size())
            {
                int cnt = 0;
                double me = -1.0, rmse = 0.0;

                for (auto &p: view.undistortedPattern)
                {
                    auto ppp = p.point;
                    ppp[1] *= factor;
                    auto pp = photostation.project(ppp, cam) - p.projection;
                    if (!pp > me)
                    {
                        me = !pp;
                    }
                    rmse += pp & pp;
                    cnt++;
                }
                rmse = cnt ? std::sqrt(rmse / cnt) : -1.0;
                view.calibrationRmse =  rmse;
                view.calibrationMaxError = me;
            }
            else
            {
                view.calibrationRmse = -1.0;
                view.calibrationMaxError = -1.0;
            }
        }
        setupLocsIterator++;
    }
}

void CalibrationJob::computeSingleCameraErrors()
{
    for (auto& s: calibrationSetups)
    {
        for (auto& c: s)
        {
            auto cam = photostation.cameras[c.cameraId];
            auto&view= observations[c.cameraId][c.imageId];

            if (view.undistortedPattern.size())
            {
                int cnt = 0;
                double me = -1.0, rmse = 0.0;
                cam.extrinsics = view.location;

                for (auto &p: view.undistortedPattern)
                {
                    auto ppp = p.point;
                    ppp[1] *= factor;
                    auto pp = cam.project(ppp) - p.projection;
                    if (!pp > me)
                    {
                        me = !pp;
                    }
                    rmse += pp & pp;
                    cnt++;
                }
                rmse = cnt ? std::sqrt(rmse / cnt) : -1.0;
                view.singleCameraRmse =  rmse;
                view.singleCameraMaxError = me;
            }
            else
            {
                view.singleCameraRmse = -1.0;
                view.singleCameraMaxError = -1.0;
            }
        }
    }
}

void CalibrationJob::calibrate()
{
    allCalibrateSingleCamera();
    computeSingleCameraErrors();
    calibratePhotostation();
    computeCalibrationErrors();
}