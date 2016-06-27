#include <array>
#include <queue>

#include "calibrationJob.h"

#include "lmDistortionSolver.h"
#include "flatPatternCalibrator.h"
#include "photoStationCalibrator.h"
#include "qtFileLoader.h"
#include "tbbWrapper.h"
#include "log.h"
#include "vector4d.h"
#include "multicameraTriangulator.h"

#ifdef WITH_OPENCV
#include "openCvCheckerboardDetector.h"
#endif

void CalibrationJob::computeReconstructionError()
{
    L_INFO << "Starting reconstruction error computation";
    // I do not use std::tuple 'cause I want to simplify fancy-looking board-like output
    std::map<double, std::map<double, std::map<double, std::vector<std::tuple<size_t, size_t, corecvs::Vector2dd>>>>> pointCollection;
    for (size_t i = 0; i < calibrationSetups.size(); ++i)
    {
        auto& ss  = calibrationSetups[i];
        for (auto& s: ss)
        {
            auto& obs = observations[s.cameraId][s.imageId];

            for (auto& ptc: obs.sourcePattern)
                pointCollection[ptc.z()][ptc.y()][ptc.x()].emplace_back(i, s.cameraId, ptc.projection);
        }
    }

    int cnt = 0;
    for (auto& i: pointCollection)
        for (auto& j: i.second)
            cnt += (int)j.second.size();

    L_INFO << "Starting reconstruction error computation: point triangulation";
    Matrix A(cnt, 3);
    int idx = 0;
    double maxError = 0.0;
    corecvs::Vector3dd mean(0, 0, 0);
    for (auto& i : pointCollection)
    {
        for (auto& j : i.second)
        {
            for (auto& k : j.second)
            {
                corecvs::Vector3dd orig(k.first, j.first, i.first);
                MulticameraTriangulator mct;
                for (auto& pt : k.second)
                {
                    auto ps = photostation;
                    ps.setLocation(calibrationSetupLocations[std::get<0>(pt)]);
                    mct.addCamera(ps.getRawCamera((int)std::get<1>(pt)).getCameraMatrix()
                        , photostation.cameras[(int)std::get<1>(pt)].distortion.mapBackward(std::get<2>(pt)));
                }
                auto diff = mct.triangulateLM(mct.triangulate()) - orig;
                std::cout << diff << " ";
                if ((!diff) > maxError)
                    maxError = !diff;
                for (int ii = 0; ii < 3; ++ii)
                    A.a(idx, ii) = diff[ii];
                idx++;
                mean += diff;
            }
            std::cout << std::endl;
        }
    }
    mean = mean * (1.0 / cnt);
    auto ATA = A.t() * A * (1.0 / cnt);
    CORE_ASSERT_TRUE_S(ATA.w == ATA.h && ATA.w == 3);
    std::cout << "Covariation [assuming zero mean] estimate: " << std::endl << ATA << std::endl;
    std::cout << "Real mean is " << mean << std::endl;
    totalReconstructionErrorMax = maxError;
    totalReconstructionErrorRMSE= std::sqrt(ATA.a(0, 0) + ATA.a(1, 1) + ATA.a(2, 2));
}

bool CalibrationJob::detectChessBoard(corecvs::RGB24Buffer &buffer, corecvs::ObservationList &list)
{
    return detectChessBoard(buffer, nullptr, &list);
}

bool CalibrationJob::detectChessBoard(corecvs::RGB24Buffer &buffer, corecvs::SelectableGeometryFeatures &features)
{
    return detectChessBoard(buffer, &features);
}

void CalibrationJob::fit(int referenceLayerCamerasCount)
{
    std::vector<int> perm;
    for (int i = 0; i < (int)photostation.cameras.size(); ++i) {
        perm.push_back(i);
    }

    std::sort(perm.begin(), perm.end(), [&](const int &a, const int &b) {
        return photostation.cameras[a].nameId < photostation.cameras[b].nameId;
    });

    auto cameras = photostation.cameras;
    auto setups = calibrationSetups;

    for (auto& s: calibrationSetups)
        for (CalibrationSetupEntry & v: s)
            v.cameraId = perm[v.cameraId];

    for (size_t i = 0; i < cameras.size(); ++i)
    {
        observations[i] = observations[perm[i]];
        photostation.cameras[i] = cameras[perm[i]];
    }

    photostation.location.shift = corecvs::Vector3dd(0.0, 0.0, 0.0);
    photostation.location.rotor = photostation.cameras[0].extrinsics.orientation;
    corecvs::Vector3dd meanShift(0.0, 0.0, 0.0);
    for (int i = 0; i < (int)photostation.cameras.size(); ++i)
    {
        photostation.cameras[i] = photostation.getRawCamera(i);
        if (i < referenceLayerCamerasCount)
            meanShift += photostation.getRawCamera(i).extrinsics.position / (6.0);
    }
    photostation.location.rotor = corecvs::Quaternion(0.0, 0.0, 0.0, 1.0);
    photostation.location.shift = -meanShift;
    for (int i = 0; i < (int)photostation.cameras.size(); ++i)
    {
        photostation.cameras[i] = photostation.getRawCamera(i);
    }
    photostation.location.rotor = corecvs::Quaternion(0, 0, 0, 1);
    photostation.location.shift = corecvs::Vector3dd(0, 0, 0);
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
        patternDetector = new ChessboardDetector(settings.openCvDetectorParameters
            , settings.boardAlignerParams
            , settings.chessBoardCornerDetectorParams
            , settings.chessBoardAssemblerParams);

        Statistics stats;
        patternDetector->setStatistics(&stats);
        patternDetector->detectPattern(buffer);     // this function is not a thread-safe for the used Statistics object!

#ifdef WITH_TBB
        lockStatsData.lock();
#endif
            statsData.addStatistics(stats);
#ifdef WITH_TBB
        lockStatsData.unlock();
#endif
    }

    if (features)
        patternDetector->getPointData(*features);
    if (list)
        patternDetector->getPointData(*list);

    delete patternDetector;

    return (features && features->mPoints.size()) || (list && list->size());
}

struct ParallelBoardDetector
{
    void operator() (const corecvs::BlockedRange<size_t> &r) const
    {
        for (size_t i = r.begin(); i != r.end(); ++i)
        {
            auto boo = job->processState->createAutoTrackerCalculationObject();
            size_t cam = idx[i][0];
            size_t obs = idx[i][1];

            ImageData& v = job->observations[cam][obs];
            auto& psIterator = job->photostation.cameras[cam];
            if (!estimate)
            {
                const std::string& filename = distorted ? v.sourceFileName : v.undistortedFileName;
                corecvs::RGB24Buffer buffer = CalibrationJob::LoadImage(filename);
                if (!buffer.hasZeroSize())
                {
                    (distorted ? psIterator.intrinsics.distortedSize : psIterator.intrinsics.size) = corecvs::Vector2dd(buffer.w, buffer.h);
                }
                job->detectChessBoard(buffer, distorted ? v.sourcePattern : v.undistortedPattern);
                job->processState->checkToCancel();
            }
            else
            {
                for (auto & p: v.sourcePattern)
                {
                    auto pc = p;
                    pc.projection = job->photostation.cameras[cam].distortion.mapBackward(pc.projection);
                    v.undistortedPattern.push_back(pc);

                    job->processState->checkToCancel();
                }
            }
        }
    }
    ParallelBoardDetector(CalibrationJob* job, std::vector<std::array<size_t, 2>> idx, bool estimate, bool distorted)
        : job(job), idx(idx), estimate(estimate), distorted(distorted)
    {}

    CalibrationJob*                     job;
    std::vector<std::array<size_t, 2>>  idx;
    bool                                estimate, distorted;
};

void CalibrationJob::allDetectChessBoard(bool distorted)
{
    int N = (int)observations.size();

    photostation.cameras.resize(N);

    bool estimate = !distorted && settings.openCvDetectorParameters.mEstimateUndistortedFromDistorted;

    L_INFO_P("chessboard type: %s", settings.boardAlignerParams.boardMarkers.size() ? "new" : "old");

    std::vector<std::array<size_t, 2>> idxs;
    for (size_t i = 0; i < observations.size(); ++i)
    {
        for (size_t j = 0; j < observations[i].size(); ++j)
        {
            idxs.emplace_back(std::array<size_t, 2>({i, j}));
        }
    }

    processState->reset("Pattern detection", idxs.size());

    // If you do not have tons of ram, then you should probably make this loop sequential // TODO: make it param/automatic
    bool shouldParallel = true;
    corecvs::parallelable_for((size_t)0, idxs.size(), ParallelBoardDetector(this, idxs, estimate, distorted), shouldParallel);

    if (!distorted && calibrated)
        computeCalibrationErrors();
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
            auto boo = job->processState->createAutoTrackerCalculationObject();

            corecvs::SelectableGeometryFeatures sgf;
            for (auto& v: job->observations[cam])
            {
                sgf.addAllLinesFromObservationList(v.sourcePattern);
                job->processState->checkToCancel();
            }

            auto& psIterator = job->photostation.cameras[cam];
            job->estimateDistortion(sgf, psIterator.intrinsics.distortedSize[0], psIterator.intrinsics.distortedSize[1], psIterator.distortion);

            for (auto& v: job->observations[cam])
            {
                job->computeDistortionError(v.sourcePattern, psIterator.distortion, v.distortionRmse, v.distortionMaxError);
                job->processState->checkToCancel();
            }
        }
    }

    ParallelDistortionEstimator(CalibrationJob *job) : job(job)
    {}

    CalibrationJob *job;
};

void CalibrationJob::allEstimateDistortion()
{
    processState->reset("Distortion estimation", photostation.cameras.size());

    corecvs::parallelable_for(0, (int)photostation.cameras.size(), ParallelDistortionEstimator(this));
}


void CalibrationJob::prepareUndistortionTransformation(int camId, corecvs::DisplacementBuffer &result)
{
    auto& cam = photostation.cameras[camId];
    cam.estimateUndistortedSize(settings.distortionApplicationParameters);

    processState->checkToCancel();

    int newW = (int)cam.intrinsics.size[0];
    int newH = (int)cam.intrinsics.size[1];
    if (newH < 0 || newW < 0)
    {
        L_ERROR_P("invalid distortion data for camId=%d outSize(%dx%d)", camId, newW, newH);
        return;
    }

    result = RadialCorrection(cam.distortion).getUndistortionTransformation(cam.intrinsics.size
        , cam.intrinsics.distortedSize, 0.25, false);
}

void CalibrationJob::removeDistortion(corecvs::RGB24Buffer &src, corecvs::RGB24Buffer &dst
    , corecvs::DisplacementBuffer &transform, double outW, double outH)
{
    corecvs::RGB24Buffer* buffer = src.doReverseDeformationBlTyped<corecvs::DisplacementBuffer>(&transform, outH, outW);
    if (buffer != NULL)
    {
        dst = *buffer;
        delete buffer;
    }
}

struct ParallelDistortionRemoval
{
    void operator() (const corecvs::BlockedRange<int> &r) const
    {
        for (int camId = r.begin(); camId < r.end(); ++camId)
        {
            auto boo = job->processState->createAutoTrackerCalculationObject();

            auto& observationsIterator = job->observations[camId];
            auto& cam = job->photostation.cameras[camId];

            corecvs::DisplacementBuffer transform;
            job->prepareUndistortionTransformation(camId, transform);

            job->processState->checkToCancel();

            corecvs::parallelable_for(0, (int)observationsIterator.size(), [&](const corecvs::BlockedRange<int> &r)
                {
                    for (int i = r.begin(); i != r.end(); ++i)
                    {
                        job->processState->checkToCancel();

                        auto &ob = observationsIterator[i];
                        corecvs::RGB24Buffer source = job->LoadImage(ob.sourceFileName), dst;
                        job->removeDistortion(source, dst, transform, cam.intrinsics.size[0], cam.intrinsics.size[1]);
                        job->SaveImage(ob.undistortedFileName, dst);
                    }
                });
        }
    }

    ParallelDistortionRemoval(CalibrationJob *job) : job(job)
    {}

    CalibrationJob *job;
};

void CalibrationJob::allRemoveDistortion()
{
    processState->reset("Image undistortion", photostation.cameras.size());
    corecvs::parallelable_for(0, (int)photostation.cameras.size(), ParallelDistortionRemoval(this));
}

void CalibrationJob::allRemoveDistortionForPattern()
{
    processState->reset("Pattern undistortion", photostation.cameras.size());
    std::vector<std::array<size_t, 2>> idxs;
    for (size_t i = 0; i < observations.size(); ++i)
    {
        for (size_t j = 0; j < observations[i].size(); ++j)
        {
            idxs.emplace_back(std::array<size_t, 2>({i, j}));
        }
    }

    corecvs::parallelable_for((size_t)0,
                              idxs.size(), ParallelBoardDetector(this, idxs, false, false), true);
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

    FlatPatternCalibrator calibrator(settings.singleCameraCalibratorConstraints
        , photostation.cameras[cameraId].intrinsics
        , settings.distortionEstimationParameters);

    bool usingUndistorted = !(settings.singleCameraCalibratorConstraints & CameraConstraints::UNLOCK_DISTORTION);

    for (auto& o: observations[cameraId])
    {
        ObservationList patternPoints;
        for (PointObservation& p: usingUndistorted ? o.undistortedPattern : o.sourcePattern)
            patternPoints.emplace_back(PointObservation(p.point, p.projection));

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
        if (!!(settings.singleCameraCalibratorConstraints & CameraConstraints::UNLOCK_DISTORTION))
        {
            photostation.cameras[cameraId].distortion = calibrator.getDistortion();
        }
        locations = calibrator.getExtrinsics();

        for (auto& o: observations[cameraId])
        {
            if ((usingUndistorted ? o.undistortedPattern : o.sourcePattern).size())
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
        {
            job->processState->checkToCancel();
            job->calibrateSingleCamera(cameraId);
        }
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

void CalibrationJob::calibratePhotostation(int N, int /*M*/, PhotoStationCalibrator &calibrator
    , std::vector<MultiCameraPatternPoints> &points
    , std::vector<PinholeCameraIntrinsics> &intrinsics
    , std::vector<LensDistortionModelParameters> &distortions
    , std::vector<std::vector<CameraLocationData>> &locations
    , bool runBFS
    , bool runLM)
{
    for (int i = 0; i < N; ++i)
    {
        if (!!(settings.photostationCalibratorConstraints & CameraConstraints::UNLOCK_DISTORTION))
            calibrator.addCamera(intrinsics[i], distortions[i]);
        else
            calibrator.addCamera(intrinsics[i]);
    }
    calibrator.factor = factor;
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
    bool usingUndistorted = !(settings.photostationCalibratorConstraints & CameraConstraints::UNLOCK_DISTORTION);

    for (int i = 0; i < M; ++i)
    {
        points[i].resize(N);
        for (auto& s: calibrationSetups[i])
        {
            points[i][s.cameraId].clear();

            auto& observationList = usingUndistorted ?
                observations[s.cameraId][s.imageId].undistortedPattern :
                observations[s.cameraId][s.imageId].sourcePattern;

            for (PointObservation& p : observationList)
                points[i][s.cameraId].emplace_back(PointObservation(p.point, p.projection));
        }
    }

    std::vector<PinholeCameraIntrinsics> intrinsics;
    std::vector<LensDistortionModelParameters> distortions;
    for (auto& c: photostation.cameras)
    {
        intrinsics.push_back(c.intrinsics);
        distortions.push_back(c.distortion);
    }
    std::vector<std::vector<CameraLocationData>> locations(M);
    for (int i = 0; i < M; ++i)
    {
        locations[i].resize(N);
        for (auto &o: calibrationSetups[i])
        {
            locations[i][o.cameraId] = observations[o.cameraId][o.imageId].location;
        }
    }

    PhotoStationCalibrator calibrator(settings.photostationCalibratorConstraints, settings.distortionEstimationParameters);
    calibratePhotostation(N, M, calibrator, points, intrinsics, distortions, locations
        , settings.photostationCalibratorUseBFSPresolver
        , settings.photostationCalibratorUseLMSolver);

    Photostation ps = calibrator.getPhotostation();
    for (int i = 0; i < N; ++i)
    {
        photostation.cameras[i].intrinsics = ps.cameras[i].intrinsics;
        photostation.cameras[i].extrinsics = ps.cameras[i].extrinsics;
        if (!!(settings.photostationCalibratorConstraints & CameraConstraints::UNLOCK_DISTORTION))
            photostation.cameras[i].distortion = ps.cameras[i].distortion;
    }

    calibrationSetupLocations = calibrator.getCalibrationSetups();
    calibrated = true;
}

void CalibrationJob::computeCalibrationErrors()
{
    double rmseTotal = 0.0, maxTotal = 0.0;
    int cntTotal = 0;
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
                    if ((!pp) > me)
                    {
                        me = !pp;
                    }
                    if ((!pp) > maxTotal)
                    {
                        maxTotal = !pp;
                    }
                    rmse += pp & pp;
                    rmseTotal += pp & pp;
                    cnt++;
                    cntTotal++;
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
    totalCalibrationErrorMax = maxTotal;
    totalCalibrationErrorRMSE = cntTotal ? std::sqrt(rmseTotal / cntTotal) : -1.0;
}

void CalibrationJob::computeFullErrors()
{
    double rmseTotal = 0.0, maxTotal = 0.0;
    int cntTotal = 0;
    auto setupLocsIterator = calibrationSetupLocations.begin();
    for (auto& s: calibrationSetups)
    {
        auto loc = *setupLocsIterator;
        photostation.setLocation(loc);
        for (auto& v: s)
        {
            auto& view = observations[v.cameraId][v.imageId];
            int cam = v.cameraId;

            if (view.sourcePattern.size())
            {
                int cnt = 0;
                double me = -1.0, rmse = 0.0;

                for (auto &p: view.sourcePattern)
                {
                    auto ppp = p.point;
                    ppp[1] *= factor;
                    auto pp = photostation.project(ppp, cam);
                    auto cp = photostation.cameras[v.cameraId].distortion.mapForward(pp) - p.projection;
                    if ((!cp) > me)
                    {
                        me = !cp;
                    }
                    if ((!cp) > maxTotal)
                    {
                        maxTotal = !cp;
                    }
                    rmse += cp & cp;
                    rmseTotal += cp & cp;
                    cnt++;
                    cntTotal++;
                }
                rmse = cnt ? std::sqrt(rmse / cnt) : -1.0;
                view.fullCameraRmse = rmse;
                view.fullCameraMaxError = me;
            }
            else
            {
                view.fullCameraRmse = -1.0;
                view.fullCameraMaxError = -1.0;
            }
        }
        setupLocsIterator++;
    }
    totalFullErrorMax = maxTotal;
    totalFullErrorRMSE = cntTotal ? std::sqrt(rmseTotal / cntTotal) : -1.0;
}


void CalibrationJob::computeSingleCameraErrors()
{
    for (auto& s: calibrationSetups)
    {
        for (auto& c: s)
        {
            CameraModel cam = photostation.cameras[c.cameraId];
            ImageData &view = observations[c.cameraId][c.imageId];

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
                    if ((!pp) > me)
                    {
                        me = !pp;
                    }
                    rmse += pp & pp;
                    cnt++;
                }
                rmse = cnt ? std::sqrt(rmse / cnt) : -1.0;
                view.singleCameraRmse = rmse;
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
    processState->reset("Calibration", 6);

    processState->incrementStarted();
    allCalibrateSingleCamera();
    processState->incrementCompleted();

    processState->incrementStarted();
    computeSingleCameraErrors();
    processState->incrementCompleted();

    processState->incrementStarted();
    calibratePhotostation();
    processState->incrementCompleted();

    processState->incrementStarted();
    computeCalibrationErrors();
    processState->incrementCompleted();

    processState->incrementStarted();
    computeFullErrors();
    processState->incrementCompleted();

    processState->incrementStarted();
    computeReconstructionError();
    processState->incrementCompleted();
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

struct CommonPlaneNormalizer : FunctionArgs
{
    void operator() (const double *in, double *out)
    {
        corecvs::Vector3dd n(in[0], in[1], in[2]);
        double denom = !n;
        for (int i = 0; i < 4; ++i)
            out[i] = in[i] / denom;
    }
    CommonPlaneNormalizer() : FunctionArgs(4, 4)
    {}
};

struct CommonPlaneFunctor : FunctionArgs
{
    void operator() (const double* in, double *out)
    {
        corecvs::Vector4dd plane(in[0], in[1], in[2], in[3]);
        double a = plane[0], b = plane[1], c = plane[2], d = plane[3];
        int argout = 0;
        for (auto& id: topLayerIdx)
        {
            auto cc = ps->cameras[id].extrinsics.position;
            out[argout++] = (a * cc[0] + b * cc[1] + c * cc[2] + d) / std::sqrt(a * a + b * b + c * c);
        }
    }
    corecvs::Photostation* ps;
    std::vector<int> topLayerIdx;

    CommonPlaneFunctor(corecvs::Photostation *ps, std::vector<int> topLayerIdx)
        : FunctionArgs(4, (int)topLayerIdx.size())
        , ps(ps)
        , topLayerIdx(topLayerIdx)
    {}
};

struct CircleFunctor : FunctionArgs
{
    void operator() (const double* in, double *out)
    {
        double cx = in[0];
        double cy = in[1];
        double r  = in[2];
        corecvs::Vector2dd c(cx, cy);
        int argout = 0;
        for (auto& v: projections)
            out[argout++] = r - !(c - v);
    }
    std::vector<corecvs::Vector2dd> projections;

    CircleFunctor(std::vector<corecvs::Vector2dd> &topLayerProjections)
        : FunctionArgs(3, (int)topLayerProjections.size())
        , projections(topLayerProjections)
    {}
};

void CalibrationJob::reorient(const corecvs::Vector3dd T, const corecvs::Quaternion Q)
{
    corecvs::Photostation reoriented = photostation;
    reoriented.location.rotor = Q;
    reoriented.location.shift = T;

    for (size_t i = 0; i < reoriented.cameras.size(); ++i)
    {
        reoriented.cameras[i] = reoriented.getRawCamera((int)i);
    }
    reoriented.location.rotor = corecvs::Quaternion(0, 0, 0, 1);
    reoriented.location.shift = corecvs::Vector3dd(0, 0, 0);

    for (auto& sl: calibrationSetupLocations)
    {
        auto qs = sl.orientation;
        auto cs = sl.position;
        auto qsn= Q ^ qs;
        auto csn=cs - (qs.conjugated() ^ Q.conjugated()) * T;

        corecvs::Photostation ps1 = photostation;
        corecvs::Photostation ps2 = reoriented;

        photostation.setLocation(sl);
        reoriented.setLocation(CameraLocationData(csn, qsn));

        for (size_t i = 0; i < photostation.cameras.size(); ++i)
        {
            auto c1 = photostation.getRawCamera((int)i);
            auto c2 = reoriented.getRawCamera((int)i);
            CORE_ASSERT_TRUE_S((!(c1.extrinsics.position - c2.extrinsics.position)) < 1e-6);
            double df = (c1.extrinsics.orientation ^ c2.extrinsics.orientation.conjugated())[3];
            double ang = std::acos(std::min(1.0, df)) * 2.0;
            ang = std::min(ang, 2.0 * M_PI - ang);
            CORE_ASSERT_TRUE_S(ang < 1e-6);
        }
        sl.orientation = qsn;
        sl.position    = csn;
    }
    photostation = reoriented;
}

void CalibrationJob::reorient(const std::vector<int> &topLayerIdx)
{
    for (int ii = 0; ii < 10; ++ii)
    {
        CommonPlaneFunctor cpf(&photostation, topLayerIdx);
        CommonPlaneNormalizer cpn;
        corecvs::LevenbergMarquardt lm(100000);

        corecvs::Vector3dd n1 = ((photostation.cameras[topLayerIdx[1]].extrinsics.position
                                - photostation.cameras[topLayerIdx[0]].extrinsics.position)
                                ^ (photostation.cameras[topLayerIdx[2]].extrinsics.position
                                 - photostation.cameras[topLayerIdx[0]].extrinsics.position)
                                ).normalised();
        if ((!n1) == 0.0) {
            L_ERROR_P("Not enough information to perform operation");  // happens when calibration was absent
            break;
        }
        double d1 = -(n1 & photostation.cameras[topLayerIdx[0]].extrinsics.position);

        std::vector<double> input(4), output(topLayerIdx.size());
        input[0] = n1[0];
        input[1] = n1[1];
        input[2] = n1[2];
        input[3] = d1;
        lm.f = &cpf;
        lm.normalisation = &cpn;
        auto res = lm.fit(input, output);
        corecvs::Vector4dd plane(res[0], res[1], res[2], res[3]);
        corecvs::Vector3dd n(res[0], res[1], res[2]);
        plane /= !n;
        n.normalise();

        int maxIdx = 0;
        double maxAxis = std::abs(n[0]);
        for (int i = 0; i < 3; ++i)
            if (std::abs(n[i]) > maxAxis)
            {
                maxAxis = std::abs(n[i]);
                maxIdx = i;
            }

        corecvs::Vector3dd origin(0, 0, 0);
        origin[maxIdx] = -plane[3] / n[maxIdx];

        CORE_ASSERT_TRUE_S(std::abs((n & origin) + plane[3]) < 1e-6);

        std::vector<corecvs::Vector3dd> orths = {
              corecvs::Vector3dd(1, 0, 0)
            , corecvs::Vector3dd(0, 1, 0)
            , corecvs::Vector3dd(0, 0, 1)
        };
        for (int i = 0; i < 3; ++i)
            orths[i] = n ^ (orths[i] ^ n);

        std::sort(orths.begin(), orths.end(), [](const corecvs::Vector3dd &a, const corecvs::Vector3dd &b) { return (!a) > (!b); });

        corecvs::Vector3dd plane1 = (n ^ orths[0]).normalised();
        corecvs::Vector3dd plane2 = (n ^ plane1).normalised();

        CORE_ASSERT_TRUE_S(std::abs(plane1 & n) < 1e-6 && std::abs(plane2 & n) < 1e-6 && std::abs(plane1 & plane2) < 1e-6);

        std::vector<corecvs::Vector2dd> projections;
        for (auto& id: topLayerIdx)
        {
            auto pos = photostation.cameras[id].extrinsics.position;
            std::cout << photostation.cameras[id].nameId << ": " << std::abs((pos - origin) & n) << std::endl;
            auto posp = pos - ((pos - origin) & n) * n;

            CORE_ASSERT_TRUE_S(std::abs(posp[0] * plane[0] + posp[1] * plane[1] + posp[2] * plane[2] + plane[3]) < 1e-6);
            double x = (posp - origin) & plane1;
            double y = (posp - origin) & plane2;
            projections.emplace_back(x, y);
        }
        corecvs::Vector2dd avg(0, 0);
        for (auto& v: projections)
            avg += v / (1.0 * projections.size());

        corecvs::LevenbergMarquardt lmc(100000);
        std::vector<double> inputc(3), outputc(topLayerIdx.size());
        inputc[0] = avg[0];
        inputc[1] = avg[1];
        inputc[2] = !(projections[0] - avg);
        CircleFunctor cf(projections);
        lmc.f = &cf;
        auto resc = lmc.fit(inputc, outputc);
        std::cout << "R = " << resc[2] << std::endl;

        corecvs::Vector3dd finalOrigin = resc[0] * plane1 + resc[1] * plane2 + origin;
        corecvs::Vector3dd finalPlane1 = (projections[0][0] * plane1 + projections[0][1] * plane2).normalised();
        corecvs::Vector3dd testdPlane2 = (projections[1][0] * plane1 + projections[1][1] * plane2).normalised();
        corecvs::Vector3dd ntest = finalPlane1 ^ testdPlane2;

        if ((ntest & n) > 0) n *= -1;

        corecvs::Vector3dd finalPlane2 = n ^ finalPlane1;
        CORE_ASSERT_TRUE_S(std::abs(finalPlane1 & n) < 1e-6 && std::abs(finalPlane2 & n) < 1e-6 && std::abs(finalPlane1 & finalPlane2) < 1e-6);

        corecvs::Matrix33 R(finalPlane1[0], finalPlane1[1], finalPlane1[2],
                            finalPlane2[0], finalPlane2[1], finalPlane2[2],
                                    n[0],           n[1],           n[2]);
        CORE_ASSERT_TRUE_S(std::abs(R.det() - 1.0) < 1e-6);

        for (size_t i = 0; i < photostation.cameras.size(); ++i)
            std::cout << photostation.cameras[i].nameId << ": " << !(photostation.cameras[i].extrinsics.position - finalOrigin) << std::endl;

        corecvs::Quaternion Q = corecvs::Quaternion::FromMatrix(R).normalised();
        corecvs::Vector3dd T = -(Q * finalOrigin);

        reorient(T, Q);
    }
}
