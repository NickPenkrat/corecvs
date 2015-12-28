#include <array>
#include <queue>

#include "calibrationJob.h"

#include "lmDistortionSolver.h"
#include "flatPatternCalibrator.h"
#include "photoStationCalibrator.h"
#include "qtFileLoader.h"
#include "tbbWrapper.h"
#include "log.h"
#ifdef WITH_OPENCV
#include "openCvCheckerboardDetector.h"
#endif

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
#ifdef WITH_OPENCV
    if (settings.openCvDetectorParameters.algorithm() == CheckerboardDetectionAlgorithm::OPENCV_DETECTOR)
    {
        patternDetector = new OpenCvCheckerboardDetector(settings.openCvDetectorParameters, settings.boardAlignerParams);
        G8Buffer* channel = buffer.getChannel(settings.openCvDetectorParameters.channel());
        patternDetector->detectPattern(*channel);
        delete channel;
    }
    else
#endif
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
    std::vector<corecvs::CameraModel>::iterator psIterator = photostation.cameras.begin();
    bool estimate = !distorted && settings.openCvDetectorParameters.mEstimateUndistortedFromDistorted;

    if (estimate)
    {
        prepareAllRadialCorrections();
    }

    L_INFO_P("chessboard type: %s", this->settings.boardAlignerParams.boardMarkers.size() ? "new" : "old");

    int camId = 0;
    for (auto& c: observations)
    {
        for (auto& v: c)
        {
            if (!estimate)
            {
                const std::string& filename = distorted ? v.sourceFileName : v.undistortedFileName;
                corecvs::RGB24Buffer buffer = LoadImage(filename);
                if (buffer.h && buffer.w)
                {
                    (distorted ? psIterator->intrinsics.distortedSize : psIterator->intrinsics.size) = corecvs::Vector2dd(buffer.w, buffer.h);
                }
                detectChessBoard(buffer, distorted ? v.sourcePattern : v.undistortedPattern);
            }
            else
            {
                for (auto&p: v.sourcePattern)
                {
                    auto pc = p;
                    pc.projection = corrections[camId].map(pc.projection);
                    v.undistortedPattern.push_back(pc);
                }
            }
        }
        ++psIterator;
        ++camId;
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

void CalibrationJob::prepareRadialCorrection(LensDistortionModelParameters &source, double w, double h, RadialCorrection &correction, double &newW, double &newH, Rect &output)
{
    correction = RadialCorrection(source);
    auto undistParams = settings.distortionApplicationParameters;
    if (settings.distortionApplicationParameters.forceScale())
    {
        correction.mParams.setScale(source.scale());
    }

    Rect input = { corecvs::Vector2dd(0.0, 0.0), corecvs::Vector2dd(w, h) };
    Rect outCir, outIns;

    correction.getCircumscribedImageRect(input[0][0], input[0][1], input[1][0], input[1][1], outCir[0], outCir[1]);
    correction.getInscribedImageRect    (input[0][0], input[0][1], input[1][0], input[1][1], outIns[0], outIns[1]);

    output = input;
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

    corecvs::Vector2dd wh = output[1] - output[0];
    newW = wh[0];
    newH = wh[1];

    if (newW < 10 || newH < 10)
    {
        L_ERROR_P("invalid undistorted output size: %dx%d", roundUp(newW), roundUp(newH));
        newW = newH = 0;
        return;
    }

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

}

void CalibrationJob::prepareAllRadialCorrections()
{
    double foo, boo;
    Rect moo;
    corrections.resize(photostation.cameras.size());
    for (size_t i = 0; i < photostation.cameras.size(); ++i)
    {
        auto& cam = photostation.cameras[i];
        prepareRadialCorrection(cam.distortion, cam.intrinsics.distortedSize[0], cam.intrinsics.distortedSize[1], corrections[i], foo, boo, moo);
    }
}

void CalibrationJob::prepareUndistortionTransformation(LensDistortionModelParameters &source, double w, double h
    , corecvs::DisplacementBuffer &result, double &newW, double &newH)
{
    RadialCorrection correction;
    Rect output;
    prepareRadialCorrection(source, w, h, correction, newW, newH, output);
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
        using namespace corecvs;
        for (int camId = r.begin(); camId < r.end(); ++camId)
        {
            auto& observationsIterator = job->observations[camId];
            auto& cam = job->photostation.cameras[camId];

            corecvs::DisplacementBuffer transform;
            Vector2dd newSize;
            job->prepareUndistortionTransformation(cam.distortion,
                    cam.intrinsics.distortedSize.x(),
                    cam.intrinsics.distortedSize.y(), transform, newSize.x(), newSize.y());

            cam.intrinsics.size = newSize;

            for (ImageData& ob: observationsIterator)
            {
                RGB24Buffer source = job->LoadImage(ob.sourceFileName), dst;
                job->removeDistortion(source, dst, transform, newSize.x(), newSize.y());
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
    corecvs::RGB24Buffer* buffer = QTRGB24Loader().load(path);

    if (buffer == NULL)
        return corecvs::RGB24Buffer();

    corecvs::RGB24Buffer res = *buffer;
    delete buffer;
    return res;
}

bool CalibrationJob::calibrateSingleCamera(int cameraId)
{
    std::vector<CameraLocationData> locations;
    int valid_locations = 0;

    FlatPatternCalibrator calibrator(settings.singleCameraCalibratorConstraints, photostation.cameras[cameraId].intrinsics);

    for (auto& o: observations[cameraId])
    {
        ObservationList patternPoints;
        for (PointObservation& p: o.undistortedPattern) {
            patternPoints.emplace_back(PointObservation(p.point, p.projection));
        }

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
        calibrator.solve(settings.singleCameraCalibratorUseZhangPresolver, settings.singleCameraCalibratorUseLMSolver, settings.singleCameraLMiterations);

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
    for (double& f: factors) {
        fac += f;
    }
    fac /= factors.size();
    factor = fac;
    std::cout << "OPTFAC_MEAN: " << factor << std::endl;
}

void CalibrationJob::calibratePhotostation(int N, int /*M*/, PhotoStationCalibrator &calibrator, std::vector<MultiCameraPatternPoints> &points, std::vector<PinholeCameraIntrinsics> &intrinsics, std::vector<std::vector<CameraLocationData>> &locations, bool runBFS, bool runLM)
{
    for (auto& ci : intrinsics) {
        calibrator.addCamera(ci);
    }
    calibrator.factor = factor;
    std::vector<int> cnt(N);
    int set = 0;
    for (auto& setup : points)
    {
        MultiCameraPatternPoints pts;
        std::vector<int> active;
        std::vector<CameraLocationData> locs;
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
        calibrator.solve(false, true, settings.photostationLMiterations);
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
            for (PointObservation& p: observations[s.cameraId][s.imageId].undistortedPattern)
                points[i][s.cameraId].emplace_back(PointObservation(p.point, p.projection));
        }
    }

    std::vector<PinholeCameraIntrinsics> intrinsics;
    for (auto& c: photostation.cameras)
        intrinsics.push_back(c.intrinsics);
    std::vector<std::vector<CameraLocationData>> locations(M);
    for (int i = 0; i < M; ++i)
    {
        locations[i].resize(N);
        for (auto &o: calibrationSetups[i])
        {
            locations[i][o.cameraId] = observations[o.cameraId][o.imageId].location;
        }
    }

    PhotoStationCalibrator calibrator(settings.photostationCalibratorConstraints);
    calibratePhotostation(N, M, calibrator, points, intrinsics, locations
        , settings.photostationCalibratorUseBFSPresolver
        , settings.photostationCalibratorUseLMSolver);

    Photostation ps = calibrator.getPhotostation();
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
        CameraLocationData loc = *setupLocsIterator;
        photostation.setLocation(loc);
        for (CalibrationSetupEntry& v: s)
        {
            ImageData& view = observations[v.cameraId][v.imageId];
            int cam = v.cameraId;

            if (view.undistortedPattern.size())
            {
                int cnt = 0;
                double me = -1.0;
                double rmse = 0.0;

                for (PointObservation &p: view.undistortedPattern)
                {
                    Vector3dd ppp = p.point;
                    ppp.y() *= factor;
                    Vector2dd pp = photostation.project(ppp, cam) - p.projection;
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
            CameraModel cam = photostation.cameras[c.cameraId];
            ImageData &view= observations[c.cameraId][c.imageId];

            if (view.undistortedPattern.size())
            {
                int cnt = 0;
                double me = -1.0, rmse = 0.0;
                cam.extrinsics = view.location;

                for (PointObservation &p: view.undistortedPattern)
                {
                    Vector3dd ppp = p.point;
                    ppp.y() *= factor;
                    Vector2dd pp = cam.project(ppp) - p.projection;
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

void CalibrationJob::calculateRedundancy(std::vector<int> &cameraImagesCount
    , std::vector<std::vector<int>> &cameraCameraRelationships
    , std::vector<int> &redundantSingleCamera
    , int &redundancyPhotostation)
{
    cameraImagesCount.clear();
    cameraImagesCount.resize(photostation.cameras.size());
    size_t id = 0;
    for (auto& o: observations)
    {
        for (auto& oo: o)
        {
            if (oo.undistortedPattern.size())
                cameraImagesCount[id]++;
        }
        id++;
    }
    redundantSingleCamera.clear();
    redundantSingleCamera.resize(cameraImagesCount.size());

    for (size_t i = 0; i < cameraImagesCount.size(); ++i)
    {
        int required = 2;
        if (!(settings.singleCameraCalibratorConstraints & corecvs::CameraConstraints::LOCK_FOCAL))
            required++;
        redundantSingleCamera[i] = cameraImagesCount[i] >= required;
    }

    cameraCameraRelationships.clear();
    cameraCameraRelationships.resize(cameraImagesCount.size());
    size_t total = 0;
    for (auto& s: calibrationSetups)
    {
        for (auto& c1: s)
        {
            if (!observations[c1.cameraId][c1.imageId].undistortedPattern.size())
                continue;
            total ++;
            for (auto& c2: s)
            {
                if (!observations[c2.cameraId][c2.imageId].undistortedPattern.size())
                    continue;
                if (&c1 == &c2) continue;
                cameraCameraRelationships[c1.cameraId].push_back(c2.cameraId);
            }
        }
    }
    for (auto& v: cameraCameraRelationships)
    {
        std::sort(v.begin(), v.end());
        if (!v.size())
            continue;
        size_t id = 1;
        for (size_t i = 1; i < v.size(); ++i)
            if (v[i] != v[id - 1])
            {
                v[id++] = v[i];
            }
        v.resize(id);
    }

    std::vector<int> used(cameraImagesCount.size());
    std::queue<int> bfsQueue;
    bfsQueue.push(0);
    used[0] = 1;
    while (bfsQueue.size())
    {
        int u = bfsQueue.front(); bfsQueue.pop();
        for (auto& cam: cameraCameraRelationships[u])
            if (!used[cam])
            {
                used[cam] = 1;
                bfsQueue.push(cam);
            }
    }

    int unvisited = 0;
    for (auto& i : used) {
        if (!i)
        {
            unvisited++;
        }
    }

    redundancyPhotostation = unvisited ? -unvisited : (int)total - ((int)cameraImagesCount.size() - 1);
}
