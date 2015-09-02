#include "calibrationJob.h"

#include "openCvCheckerboardDetector.h"
#include "lmDistortionSolver.h"

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
    if (settings.useOpenCVDetector)
    {
        patternDetector = new OpenCvCheckerboardDetector(settings.openCvDetectorParameters);
        G8Buffer* channel = buffer.getChannel(settings.openCvDetectorParameters.channel());
        patternDetector->detectPattern(*channel);
        delete channel;
    }
    else
    {
        patternDetector = new ChessboardDetector(settings.chessBoardDetectorParams, settings.chessBoardCornerDetectorParams, settings.chessBoardAssemblerParams);
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
    int N = observations.size();
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
                psIterator->intrinsics.size = corecvs::Vector2dd(buffer.w, buffer.h);
            }
            detectChessBoard(buffer, distorted ? v.sourcePattern : v.undistortedPattern);
        }
        ++psIterator;
    }
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

void CalibrationJob::allEstimateDistortion()
{
    // TODO: check if distortion estimator is parallelized and add TBB parallelizator if it is not
    auto psIterator = photostation.cameras.begin();
    for (auto& c: observations)
    {
        corecvs::SelectableGeometryFeatures sgf;
        for (auto& v: c)
        {
            sgf.addAllLinesFromObservationList(v.sourcePattern);
        }

        estimateDistortion(sgf, psIterator->intrinsics.size[0], psIterator->intrinsics.size[1], psIterator->distortion);
        ++psIterator;
    }
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
    corecvs::Vector2dd shift;

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

    result = DisplacementBuffer::CacheInverse(
            &correction, newH, newW,
            output[0][0], output[0][1],
            output[1][0], output[1][1],
            0.25, 0.0);
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

void CalibrationJob::allRemoveDistortion()
{
    auto observationsIterator = observations.begin();
    for (auto& cam: photostation.cameras)
    {
        corecvs::DisplacementBuffer transform;
        double newW, newH;
        prepareUndistortionTransformation(cam.distortion, cam.intrinsics.size[0], cam.intrinsics.size[1], transform, newW, newH);
        for (auto& ob: *observationsIterator)
        {
            corecvs::RGB24Buffer source = LoadImage(ob.sourceFileName), dst;
            removeDistortion(source, dst, transform, newW, newH);
            SaveImage(ob.undistortedFileName, dst);
        }
        ++observationsIterator;
    }
}
