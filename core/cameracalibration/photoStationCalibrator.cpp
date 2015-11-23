#include "photoStationCalibrator.h"
#include "tbbWrapper.h"

#include "global.h"

#include <queue>
#include <algorithm>

PhotoStationCalibrator::PhotoStationCalibrator(CameraConstraints constraints, const double lockFactor) : factor(lockFactor), N(0), M(0), K(0), L(0), constraints(constraints)
{
}

void PhotoStationCalibrator::addCamera(PinholeCameraIntrinsics &intrinsics)
{
    N++;
    relativeCameraPositions.push_back({intrinsics, CameraLocationData()});
}


void PhotoStationCalibrator::addCalibrationSetup(std::vector<int> &cameraIds, std::vector<CameraLocationData> &cameraLocations, MultiCameraPatternPoints &points)
{
    patternPoints.resize(M + 1);
    initialGuess.resize(M + 1);
    absoluteSetupLocation.resize(M + 1);
    patternPoints[M].resize(N);
    initialGuess[M].resize(N);

    CORE_ASSERT_TRUE_S(cameraLocations.size() == points.size());
    CORE_ASSERT_TRUE_S(cameraIds.size() == points.size());

    for (size_t i = 0; i < cameraIds.size(); ++i)
    {
        int idx = cameraIds[i];
        patternPoints[M][idx] = points[i];
        K += (int)points[i].size();
        ++L;
        cameraLocations[i].orientation.normalise();
        initialGuess[M][idx] = std::make_pair(true, cameraLocations[i]);
    }
    ++M;
}

void PhotoStationCalibrator::solve(bool runPresolver, bool runNonLinear, int LMiterations)
{
    solve(runPresolver, runNonLinear, constraints, LMiterations);
}

void PhotoStationCalibrator::solve(bool runPresolver, bool runNonLinear, CameraConstraints constraints, int LMiterations)
{
    this->constraints = constraints;
    if (runPresolver)
    {
       solveInitialLocations();
    }
    if (runNonLinear)
    {
        refineGuess(LMiterations);
    }
    std::cout << "OPTFAC_ALL: " << factor << std::endl;
}

std::vector<CameraLocationData> PhotoStationCalibrator::getCalibrationSetups()
{
    return absoluteSetupLocation;
}

Photostation PhotoStationCalibrator::getPhotostation()
{
    return Photostation(relativeCameraPositions, CameraLocationData());
}

double PhotoStationCalibrator::getRmseReprojectionError()
{
    std::vector<double> err(2 * K);
    getFullReprojectionError(&err[0]);

    double sqs = 0.0;
    for (auto e: err) sqs += e * e;

    return sqrt(sqs / K);
}

void PhotoStationCalibrator::getFullReprojectionError(std::vector<double> &err)
{
    err.resize(2 * K);
    getFullReprojectionError(&err[0]);
}

void PhotoStationCalibrator::getFullReprojectionError(double out[])
{
    std::vector<int> offset(M);
    for (size_t j = 0; j + 1 < (size_t)M; ++j)
    {
        size_t K0 = 0;
        for (size_t i = 0; i < (size_t)N; ++i)
        {
            K0 += patternPoints[j][i].size() * 2;
        }
        offset[j + 1] = (int)K0 + offset[j];
    }

    corecvs::parallelable_for (0, M, 1, ParallelErr(this, &offset, out));
}

#define IFNOT(cond, expr) \
    if (!(constraints & CameraConstraints::cond)) \
    { \
        expr; \
    }
#define GET_PARAM(ref) \
    ref = in[argin++];
#define IF_GET_PARAM(cond, ref) \
    if (!!(constraints & CameraConstraints::cond)) ref = in[argin++];
#define IFNOT_GET_PARAM(cond, ref) \
    if (!(constraints & CameraConstraints::cond)) ref = in[argin++];
#define SET_PARAM(ref) \
    out[argout++] = ref;
#define IF_SET_PARAM(cond, ref) \
    if (!!(constraints & CameraConstraints::cond)) out[argout++] = ref;
#define IFNOT_SET_PARAM(cond, ref) \
    if (!(constraints & CameraConstraints::cond)) out[argout++] = ref;

int PhotoStationCalibrator::getInputNum() const
{
    // M setup locations = M * 7 params
    // (N-1) camera locations = (N-1) * 7 params
    // tricky magick for intrinsics
    //
    int input = 0;
    IFNOT(LOCK_FOCAL,
            input++;
            IFNOT(EQUAL_FOCAL,
                input++));
    IFNOT(LOCK_PRINCIPAL, input += 2);
    IFNOT(LOCK_SKEW, IFNOT(ZERO_SKEW, input++));
    input = input * N;
    input += M * 7;
    input += (N - 1) * 7;
    input++;
    IFNOT(UNLOCK_YSCALE, input--);
    return input;
}

int PhotoStationCalibrator::getOutputNum() const
{
    return 2 * K
#ifdef PENALIZE_QNORM
        + N - 1 + M
#endif
        ;
}

int PhotoStationCalibrator::getStructInputNum() const
{
    return M * 7 + (N - 1) * 7;
}

int PhotoStationCalibrator::getStructOutputNum() const
{
    return L * 4;
}


// TODO: lots of copies back and forth. Probably we need a double** version of LM-optimization?!
void PhotoStationCalibrator::readParams(const double in[])
{
    int argin = 0;
    for (int i = 0; i < N; ++i)
    {
        CameraModel &toFill = relativeCameraPositions[i];

        IFNOT(LOCK_FOCAL,
                double f;
                GET_PARAM(f);
                toFill.intrinsics.focal = Vector2dd(f, f);
                IFNOT_GET_PARAM(EQUAL_FOCAL, toFill.intrinsics.focal.y()));
        IFNOT(LOCK_PRINCIPAL,
                GET_PARAM(toFill.intrinsics.principal.x());
                GET_PARAM(toFill.intrinsics.principal.y()));
        IFNOT(LOCK_SKEW,
                IFNOT_GET_PARAM(ZERO_SKEW, toFill.intrinsics.skew));
        if (i > 0)
        {
            for (int j = 0; j < 3; ++j)
                GET_PARAM(toFill.extrinsics.position[j]);
            for (int j = 0; j < 4; ++j)
            {
                GET_PARAM(toFill.extrinsics.orientation[j]);
            }
            toFill.extrinsics.orientation.normalise();
        }
    }
    for (int i = 0; i < M; ++i)
    {
        for (int j = 0; j < 3; ++j)
            GET_PARAM(absoluteSetupLocation[i].position[j]);
        for (int j = 0; j < 4; ++j)
        {
            GET_PARAM(absoluteSetupLocation[i].orientation[j]);
        }
        absoluteSetupLocation[i].orientation.normalise();
    }

    IF_GET_PARAM(UNLOCK_YSCALE, factor);

    CORE_ASSERT_TRUE_S(getInputNum() == argin);
}

void PhotoStationCalibrator::writeParams(double out[])
{
    int argout = 0;
    for (int i = 0; i < N; ++i)
    {
       CameraModel &toWrite = relativeCameraPositions[i];

        IFNOT(LOCK_FOCAL,
                SET_PARAM(toWrite.intrinsics.focal.x());
                IFNOT_SET_PARAM(EQUAL_FOCAL, toWrite.intrinsics.focal.y()));
        IFNOT(LOCK_PRINCIPAL,
                SET_PARAM(toWrite.intrinsics.principal.x());
                SET_PARAM(toWrite.intrinsics.principal.y()));
        IFNOT(LOCK_SKEW,
                IFNOT_SET_PARAM(ZERO_SKEW, relativeCameraPositions[i].intrinsics.skew));
        if (i > 0)
        {
            for (int j = 0; j < 3; ++j)
            {
                SET_PARAM(toWrite.extrinsics.position[j]);
            }
            for (int j = 0; j < 4; ++j)
            {
                SET_PARAM(toWrite.extrinsics.orientation[j]);
            }
        }
    }

    for (int i = 0; i < M; ++i)
    {
        for (int j = 0; j < 3; ++j)
            SET_PARAM(absoluteSetupLocation[i].position[j]);
        for (int j = 0; j < 4; ++j)
            SET_PARAM(absoluteSetupLocation[i].orientation[j]);
    }

    IF_SET_PARAM(UNLOCK_YSCALE, factor);

    CORE_ASSERT_TRUE_S(argout == getInputNum());
}

void PhotoStationCalibrator::readStructureParams(const double in[])
{
    int argin = 0;
    for (int i = 0; i < M; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            GET_PARAM(absoluteSetupLocation[i].position[j]);
        }
        for (int j = 0; j < 4; ++j)
        {
            GET_PARAM(absoluteSetupLocation[i].orientation[j]);
        }
        absoluteSetupLocation[i].orientation.normalise();
    }
    for (int i = 1; i < N; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            GET_PARAM(relativeCameraPositions[i].extrinsics.position[j]);
        }
        for (int j = 0; j < 4; ++j)
        {
            GET_PARAM(relativeCameraPositions[i].extrinsics.orientation[j]);
        }
        relativeCameraPositions[i].extrinsics.orientation.normalise();
    }
    CORE_ASSERT_TRUE_S(argin == getStructInputNum());
}

void PhotoStationCalibrator::writeStructureParams(double out[])
{
    int argout = 0;
    for (int i = 0; i < M; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            SET_PARAM(absoluteSetupLocation[i].position[j]);
        }
        for (int j = 0; j < 4; ++j)
        {
            SET_PARAM(absoluteSetupLocation[i].orientation[j]);
        }
    }
    for (int i = 1; i < N; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            SET_PARAM(relativeCameraPositions[i].extrinsics.position[j]);
        }
        for (int j = 0; j < 4; ++j)
        {
            SET_PARAM(relativeCameraPositions[i].extrinsics.orientation[j]);
        }
    }
    CORE_ASSERT_TRUE_S(argout == getStructInputNum());
}


void PhotoStationCalibrator::LMCostFunction::operator()(const double in[], double out[])
{
    calibrator->readParams(in);
    calibrator->getFullReprojectionError(out);

#ifdef PENALIZE_QNORM
    for (size_t i = 1; i < calibrator->N; ++i)
    {
        out[2 * calibrator->K + i] = 1.0 - calibrator->relativeCameraPositions[i].extrinsics.orientation.sumAllElementsSq();
    }
    for (size_t i = 0; i < calibrator->M; ++i)
    {
        out[2 * calibrator->K + calibrator->N - 1 + i] = 1.0 - calibrator->absoluteSetupLocation[i].orientation.sumAllElementsSq();
    }
#endif
}

void PhotoStationCalibrator::LMCostFunctionNormalizer::operator()(const double in[], double out[])
{
    calibrator->readParams(in);
    calibrator->writeParams(out);
}

void PhotoStationCalibrator::LMStructureNormalizer::operator()(const double in[], double out[])
{
    calibrator->readStructureParams(in);
    calibrator->writeStructureParams(out);
}

void PhotoStationCalibrator::LMStructure::operator() (const double in[], double out[])
{
    int /*argin = 0,*/ argout = 0;
    auto& absoluteSetupLocation = calibrator->absoluteSetupLocation;
    auto& initialGuess = calibrator->initialGuess;
    auto& relativeCameraPositions = calibrator->relativeCameraPositions;
    int M = calibrator->M;
    int N = calibrator->N;
    calibrator->readStructureParams(in);

    for (int i = 0; i < M; ++i)
    {
        for (int j = 0; j < N; ++j)
        {
            if (!initialGuess[i][j].first)
                continue;

            auto& qs = absoluteSetupLocation[i].orientation;
            auto& cs = absoluteSetupLocation[i].position;

            auto& qc = relativeCameraPositions[j].extrinsics.orientation;
            auto& cc = relativeCameraPositions[j].extrinsics.position;

            auto& qr = initialGuess[i][j].second.orientation;
            auto& cr = initialGuess[i][j].second.position;

            auto diff = (qs.conjugated() * cc) + cs - cr;
            qr.normalise(); qs.normalise(); qc.normalise();
            auto diff_q = (qc ^ qs) ^ qr.conjugated();
            auto diff_angle = std::acos(std::max(-1.0, std::min(diff_q[3], 1.0))) * 2.0;
            if (diff_angle > M_PI)
                diff_angle = 2.0 * M_PI - diff_angle;
            diff_angle *= 100;
            CORE_ASSERT_TRUE_S(!std::isnan(diff_angle));
            for (int k = 0; k < 3; ++k)
            {
                CORE_ASSERT_TRUE_S(!std::isnan(diff[k]));
                out[argout++] = diff[k];
            }
            out[argout++] = diff_angle;
        }
    }
    CORE_ASSERT_TRUE_S(argout == calibrator->getStructOutputNum());
}

#undef SET_PARAM
#undef IF_SET_PARAM
#undef IFNOT_SET_PARAM
#undef IFNOT
#undef GET_PARAM
#undef IF_GET_PARAM
#undef IF_NOT_GET_PARAM


// FIXME: This should be just a linear LSQ problem and thus should be solved
// with pseudo-inverse
struct LSQCenter : public corecvs::FunctionArgs
{
    int N;
    std::vector<corecvs::Vector3dd>  centers;
    std::vector<corecvs::Quaternion> rotations;

    LSQCenter(decltype(centers) &centers, decltype(rotations) &rotations)
        : FunctionArgs(3, (int)(centers.size() * (centers.size() - 1) * 3))
        , N((int)centers.size())
        , centers(centers)
        , rotations(rotations)
    {}

    void operator()(const double in[], double out[])
    {
        corecvs::Vector3dd x(in[0], in[1], in[2]);
        int idx = 0;
        for (int i = 0; i < N; ++i)
        {
            for (int j = 0; j < N; ++j)
            {
                if (i == j)
                    continue;
                auto& qi = rotations[i];
                auto& ci = centers[i];
                auto& qj = rotations[j];
                auto& cj = centers[j];
                auto   e = qj * ((qi.conjugated() * x) + ci - cj) - x;

                for (int j = 0; j < 3; ++j)
                    out[idx++] = e[j];
            }
        }
    }
};

void PhotoStationCalibrator::recenter()
{
    std::cout << "Pre-centering RMSE: " << getRmseReprojectionError() << std::endl;
    std::vector<corecvs::Vector3dd> c;
    std::vector<corecvs::Quaternion> q;
    corecvs::Vector3dd sum;
    for (auto& l: relativeCameraPositions)
    {
        sum += l.extrinsics.position;
    }
    for (auto& l: absoluteSetupLocation)
    {
        c.push_back(l.position);
        q.push_back(l.orientation);
    }
    sum /= N;

    std::vector<double> in(3), out(c.size()*(c.size()-1) * 3);
    for (int i = 0; i < 3; ++i)
        in[i] = sum[i];

    corecvs::LevenbergMarquardt levmar(50);
    levmar.f = new LSQCenter(c, q);

    auto res = levmar.fit(in, out);
    corecvs::Vector3dd ctr(res[0], res[1], res[2]);
    std::cout << ctr << std::endl;

    for (auto& cp: relativeCameraPositions)
    {
        cp.extrinsics.position -= ctr;
    }

    for (auto& sp: absoluteSetupLocation)
    {
        sp.position += sp.orientation.conjugated() * ctr;
    }

    std::cout << "Post-recentering validation" << std::endl;
    std::cout << "Post-centering RMSE: " << getRmseReprojectionError() << std::endl;
    validate();
    std::cout << std::endl;
}

void PhotoStationCalibrator::refineGuess(int LMiterations)
{
    recenter();
    refineStruct();
    recenter();
    std::cout << "Pre-LM RMSE: " << getRmseReprojectionError() << std::endl;
    std::vector<double> in(getInputNum()), out(getOutputNum());
    writeParams(&in[0]);

    // TODO: So far, after re-implementing presolver using MST we get more robust
    //       initial estimate and get less problems from bad initialization
    //
    //       However, we can make some usefull things:
    //       * Standartizing variables
    corecvs::LevenbergMarquardt levmar(LMiterations, 10.0, 2);
    levmar.f = new LMCostFunction(this);
    levmar.normalisation = new LMCostFunctionNormalizer(this);
    levmar.trace = true;

    auto res = levmar.fit(in, out);
    readParams(&res[0]);
    std::cout << "LM validation" << std::endl;
    std::cout << "Post-LM RMSE: " << getRmseReprojectionError() << std::endl;
    validate();
    recenter();
    delete_safe(levmar.f);
    delete_safe(levmar.normalisation);
    std::cout << std::endl;
}

void PhotoStationCalibrator::refineStruct()
{
    std::cout << "Pre-struct-LM RMSE: " << getRmseReprojectionError() << std::endl;
    std::vector<double> in(getStructInputNum()), out(getStructOutputNum());
    writeStructureParams(&in[0]);

    corecvs::LevenbergMarquardt levmar(500);
    levmar.f = new LMStructure(this);
    levmar.normalisation = new LMStructureNormalizer(this);
    levmar.trace = true;
    auto res = levmar.fit(in, out);
    readStructureParams(&res[0]);

    delete_safe(levmar.f);
    delete_safe(levmar.normalisation);

    std::cout << "Post-structure validation" << std::endl;
    std::cout << "Post-struct-LM RMSE: " << getRmseReprojectionError() << std::endl;
    validate();
}

void PhotoStationCalibrator::solveCameraToSetup(const CameraLocationData &realLocation, int camera, int setup)
{
    // Camera location (absolute and relative) is known, need to solve setup params
    // Qs = Qc^{-1}*Qr
    // Cs = Cr-Qs*Cc
    corecvs::Quaternion qc = relativeCameraPositions[camera].extrinsics.orientation;
    corecvs::Quaternion qr = realLocation.orientation;
    corecvs::Vector3dd  cc = relativeCameraPositions[camera].extrinsics.position;
    corecvs::Vector3dd  cr = realLocation.position;
    qc.normalise(); qr.normalise();

    // Assuming normalization
    corecvs::Quaternion qs = qc.conjugated() ^ qr;
    corecvs::Vector3dd  cs = cr - (qs.conjugated() * cc);

    absoluteSetupLocation[setup] = CameraLocationData(cs, qs);

    corecvs::Quaternion qf = qc ^ qs;
    corecvs::Vector3dd  cf = (qs.conjugated() * cc) + cs;//qc(qs(-cs)-cc)
#ifdef VERBOSE_OUTPUT
    std::cout << "Solving setup " << setup << " from camera " << camera << " QR: " << qr << " QC: " << qc << " QS: " << qs << " | " << " CR: " << cr << " CC: " << cc << " CS: " << cs << std::endl;
#endif
    CORE_ASSERT_TRUE_S(!(cr - cf) < 1e-6);
    CORE_ASSERT_TRUE_S(!(qf - qr) < 1e-6);
    if (qs[0] > 0.0)
        qs = -qs;
#ifdef VERBOSE_OUTPUT
    std::cout << "CC:C>S: Setup " << setup << " is " << (acos(qs[3]) * 180.0 * 2.0 / M_PI) << std::endl;
#endif
}

void PhotoStationCalibrator::solveSetupToCamera(const CameraLocationData &realLocation, int camera, int setup)
{
    // Absolute camera location and setup location are known, need to solve relative camera params
    // Qc = Qr*Qs^{-1}
    // Cc = Qs*(Cr-Cs)
    corecvs::Quaternion qs = absoluteSetupLocation[setup].orientation;
    corecvs::Quaternion qr = realLocation.orientation;
    corecvs::Vector3dd  cs = absoluteSetupLocation[setup].position;
    corecvs::Vector3dd  cr = realLocation.position;
    qr.normalise(); qs.normalise();

    // Assuming normalization
    corecvs::Quaternion qc = qr ^ qs.conjugated();
    corecvs::Vector3dd  cc = qs * (cr - cs);

    relativeCameraPositions[camera].extrinsics = CameraLocationData(cc, qc);

    corecvs::Quaternion qf = qc ^ qs;
    corecvs::Vector3dd  cf = (qs.conjugated() * cc) + cs;//qc(qs(-cs)-cc)
#ifdef VERBOSE_OUTPUT
    std::cout << "Solving camera " << camera<< " from setup " << setup << " QR: " << qr << " QC: " << qc << " QS: " << qs << " | " << " CR: " << cr << " CC: " << cc << " CS: " << cs << std::endl;
#endif
    CORE_ASSERT_TRUE_S(!(cr - cf) < 1e-6);
    CORE_ASSERT_TRUE_S(!(qf - qr) < 1e-6);
    if (qc[0] > 0.0)
        qc = -qc;
#ifdef VERBOSE_OUTPUT
    std::cout << "CC:S>C: Camera " << camera << " is " << (acos(qc[3]) * 180.0 * 2.0 / M_PI) << std::endl;
#endif
}

void PhotoStationCalibrator::validate()
{
    double max_diff = 0.0;
    for (int setup = 0; setup < M; ++setup)
    {
        for (int cam = 0; cam < N; ++cam)
        {

            if (!initialGuess[setup][cam].first)
                continue;
            auto q1 = relativeCameraPositions[cam].extrinsics.orientation ^ absoluteSetupLocation[setup].orientation;
            auto q2 = initialGuess[setup][cam].second.orientation;
            auto v1 = absoluteSetupLocation[setup].position + absoluteSetupLocation[setup].orientation.conjugated() * relativeCameraPositions[cam].extrinsics.position;
            auto v2 = initialGuess[setup][cam].second.position;
            std::cout << "Cam " << cam << ", setup " << setup << std::endl;
            std::cout << "Exp: " << q2 << " Rec: " << q1 << " diff: " << (q1.conjugated() ^ q2) << std::endl;
            std::cout << "Exp: " << v2 << " Rec: " << v1 << " diff: " << v1 - v2 << std::endl;
            std::cout << "Set: " << absoluteSetupLocation[setup].position << " " << absoluteSetupLocation[setup].orientation << std::endl;

            if (max_diff < !(v2 - v1))
                max_diff = !(v2 - v1);
        }
    }
    std::cout << "Max structural diff: " << max_diff << std::endl;
}

/*
 * Here we compute maximal spanning tree from graph with verticies (cam, setup) connected
 * with edges of cost #chessboard_corners visible from setup by cam
 */
bool PhotoStationCalibrator::getMSTSolveOrder(std::vector<std::pair<int, int>> &order)
{
    std::vector<int> totalCnt(N);
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < M; ++j)
            totalCnt[i] += (int)patternPoints[j][i].size();
    int maxCam = 0;
    for (int i = 1; i < N; ++i)
        if (totalCnt[i] > totalCnt[maxCam])
            maxCam = i;
    size_t maxFC = 0;
    order.clear();
    std::pair<int, int> initial(-1, -1);
//  for (int i = 0; i < N; ++i)
    for (int j = 0; j < M; ++j) {
        if (patternPoints[j][maxCam].size() > maxFC)
        {
            maxFC = patternPoints[j][maxCam].size();
            initial = std::make_pair(maxCam, j);
        }
    }

    int cameraSolved = 1;
    int setupSolved = 0;
    std::vector<int> cameraUsed(N), setupUsed(M);
    cameraUsed[initial.first] = 1;
    setupUsed[initial.second] = 0;
    int totalS = 0, totalC = 0;
    std::vector<int> cc(N), sc(M);
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < M; ++j)
        {
            if (patternPoints[j][i].size())
            {
                cc[i]++;
                sc[j]++;
            }
        }
    for (auto& c: cc) if (c) totalC++;
    for (auto& s: sc) if (s) totalS++;
    while (setupSolved < totalS || cameraSolved < totalC)
    {
        maxFC = 0;
        initial = std::make_pair(-1, -1);
        for (int i = 0; i < M; ++i)
        {
            for (int j = 0; j < N; ++j)
            {
                if (setupUsed[i])
                {
                    for (int ii = 0; ii < N; ++ii)
                    {
                        if ((!cameraUsed[ii]) && patternPoints[i][ii].size() > maxFC)
                        {
                            initial = std::make_pair(ii, i);
                            maxFC = patternPoints[i][ii].size();
                        }
                    }
                }
                if (cameraUsed[j])
                {
                    for (int jj = 0; jj < M; ++jj)
                    {
                        if ((!setupUsed[jj]) && patternPoints[jj][j].size() > maxFC)
                        {
                            initial = std::make_pair(j, jj);
                            maxFC = patternPoints[jj][j].size();
                        }
                    }
                }

            }
        }
#ifdef VERBOSE_OUTPUT
        std::cout << "Adding " << initial.first << " " << initial.second << " : " << maxFC << std::endl;
#endif
        CORE_ASSERT_TRUE_S(patternPoints[initial.second][initial.first].size() > 0);
        CORE_ASSERT_TRUE_S(initial.first >= 0 && initial.second >= 0);
        order.push_back(initial);
        CORE_ASSERT_TRUE_S(cameraUsed[initial.first] || setupUsed[initial.second]);
        if (!cameraUsed[initial.first]) cameraSolved++;
        if (!setupUsed[initial.second]) setupSolved++;
        cameraUsed[initial.first] = 1;
        setupUsed[initial.second] = 1;
    }
#ifdef VERBOSE_OUTPUT
    for (int i = 0; i < order.size(); ++i)
    {
        std::cout << "Cam " << order[i].first << " setup " << order[i].second << " : " << patternPoints[order[i].second][order[i].first].size() << std::endl;
    }
#endif
    return true;
}

void PhotoStationCalibrator::solveInitialLocations()
{
    /*
     * Here we use MST-approach for initialization
     */
    std::vector<std::pair<int, int>> order2;
    getMSTSolveOrder(order2);
    std::vector<bool> camsSolved(N);
    std::vector<bool> setupsSolved(M);
    auto initial = order2[0];
    camsSolved[initial.first] = true;
    relativeCameraPositions[initial.first].extrinsics = corecvs::CameraLocationData(corecvs::Vector3dd(0.0, 0.0, -120.0), corecvs::Quaternion(0.0, 0.0, 0.0, 1.0));
    for (uint i = 0; i < order2.size(); ++i)
    {
        auto id = order2[i];
        std::cout << "Solving c:" << id.first << " s: " << id.second << std::endl;
        CORE_ASSERT_TRUE_S(camsSolved[id.first] || setupsSolved[id.second]);
        CORE_ASSERT_TRUE_S(!camsSolved[id.first]||!setupsSolved[id.second]);
        if (camsSolved[id.first])
        {
            solveCameraToSetup(initialGuess[id.second][id.first].second, id.first, id.second);
            setupsSolved[id.second] = true;
        }
        else
        {
            solveSetupToCamera(initialGuess[id.second][id.first].second, id.first, id.second);
            camsSolved[id.first] = true;
        }
    }
    validate();
}
