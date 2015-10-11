#include "photoStationCalibrator.h"
#include "tbbWrapper.h"

#include <queue>
#include <cassert>
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

    assert(cameraLocations.size() == points.size());
    assert(cameraIds.size() == points.size());

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

void PhotoStationCalibrator::solve(bool runPresolver, bool runNonLinear)
{
    solve(runPresolver, runNonLinear, constraints);
}

void PhotoStationCalibrator::solve(bool runPresolver, bool runNonLinear, CameraConstraints constraints)
{
    this->constraints = constraints;
    if (runPresolver)
    {
       solveInitialLocations();
    }
    if (runNonLinear)
    {
        refineGuess();
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
    //TODO: BTW it seems that not calculations but copies are slow
    //May be direct writing of error function will be smarter
#if 0
    int idx = 0;
    for (size_t j = 0; j < M; ++j)
    {
        auto& Qs = absoluteSetupLocation[j].orientation;
        auto& Cs = absoluteSetupLocation[j].position;
        for (size_t i = 0; i < N; ++i)
        {
            if (!patternPoints[j][i].size())
                continue;

            for (auto& pt: patternPoints[j][i])
            {
                auto diff = relativeCameraPositions[i].project(Qs * (pt.second - Cs));
                out[idx++] = diff[0];
                out[idx++] = diff[1];
            }
        }
    }
    assert(idx == getOutputNum() - N - M + 1);
#else
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
#endif
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
                toFill.extrinsics.orientation.normalise();
            }
        }
    }
    for (int i = 0; i < M; ++i)
    {
        for (int j = 0; j < 3; ++j)
            GET_PARAM(absoluteSetupLocation[i].position[j]);
        for (int j = 0; j < 4; ++j)
        {
            GET_PARAM(absoluteSetupLocation[i].orientation[j]);
            absoluteSetupLocation[i].orientation.normalise();
        }
    }

    IF_GET_PARAM(UNLOCK_YSCALE, factor);

    assert(getInputNum() == argin);
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

    assert(argout == getInputNum());
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

void PhotoStationCalibrator::LMStructure::operator() (const double in[], double out[])
{
    int argin = 0, argout = 0;
    auto& absoluteSetupLocation = calibrator->absoluteSetupLocation;
    auto& initialGuess = calibrator->initialGuess;
    auto& relativeCameraPositions = calibrator->relativeCameraPositions;
    int M = calibrator->M;
    int N = calibrator->N;

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

    for (int i = 0; i < M; ++i)
    {
        for (int j = 0; j < N; ++j)
        {
            if (!initialGuess[i][j].first)
                continue;

            auto& qs = absoluteSetupLocation[i].orientation;
            auto& cs = absoluteSetupLocation[i].position;

          //auto& qc = relativeCameraPositions[j].extrinsics.orientation;
            auto& cc = relativeCameraPositions[j].extrinsics.position;

          //auto& qr = initialGuess[i][j].second.orientation;
            auto& cr = initialGuess[i][j].second.position;

            auto diff = (qs.conjugated() * cc) + cs - cr;
            for (int k = 0; k < 3; ++k)
                out[argout++] = diff[k];
        }
    }
    assert(argout == calibrator->L * 3);
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

void PhotoStationCalibrator::refineGuess()
{
    std::cout << "Pre-LM RMSE: " << getRmseReprojectionError() << std::endl;
    std::vector<double> in(getInputNum()), out(getOutputNum());
    writeParams(&in[0]);

    // TODO: TIL: there is some problem in calibrator; non-linear part does not converges,
    //       it stucks somewhere at random point. Further investigation required
    //       Possible outcomes:
    //       * Adding gradient-norm related part (aka L-M instead of L) will help
    //       * Standartizing items (since we have lots of coordinates ~[-100 .. 100]
    //         and some quaternions [-1 .. 1] will help
    //       * Making BFS from multiple cameras and then taking the mean of solutions
    //         will be a better starting point for non-linear part
    //       * Some global stochastic-like procedure for minimization
    // TODO: Maybe we should test if such problem is solvable in some math package
    //       with "proven" implementations of linear algebra and ML-like non-linear
    //       optimization
    // FIXME: Changed limit to 750 since change in rmse is not big; but it still decreases!
    corecvs::LevenbergMarquardt levmar(750, 1e-3, 1.5);
    levmar.f = new LMCostFunction(this);

    auto res = levmar.fit(in, out);
    readParams(&res[0]);

    std::cout << "LM validation" << std::endl;
    std::cout << "Post-LM RMSE: " << getRmseReprojectionError() << std::endl;
    validate();
    std::cout << std::endl;
}

void PhotoStationCalibrator::refineStruct()
{
#if 1
    std::cout << "Pre-struct-LM RMSE: " << getRmseReprojectionError() << std::endl;
    std::vector<double> in(7 * M), out(3 * L);
    int argin = 0;
    for (int i = 0; i < M; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            in[argin++] = absoluteSetupLocation[i].position[j];
        }
        for (int j = 0; j < 4; ++j)
        {
            in[argin++] = absoluteSetupLocation[i].orientation[j];
        }
    }

    corecvs::LevenbergMarquardt levmar(1000, 1e-6, 1.1);
    levmar.f = new LMStructure(this);
    auto res = levmar.fit(in, out);
    argin = 0;

    for (int i = 0; i < M; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            absoluteSetupLocation[i].position[j] = res[argin++];
        }
        for (int j = 0; j < 4; ++j)
        {
            absoluteSetupLocation[i].orientation[j] = res[argin++];
        }
        absoluteSetupLocation[i].orientation.normalise();
    }
    std::cout << "Post-structure validation" << std::endl;
    std::cout << "Post-struct-LM RMSE: " << getRmseReprojectionError() << std::endl;
    validate();
#endif
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
    std::cout << "Solving setup " << setup << " from camera " << camera << " QR: " << qr << " QC: " << qc << " QS: " << qs << " | " << " CR: " << cr << " CC: " << cc << " CS: " << cs << std::endl;
    assert(!(cr - cf) < 1e-6);
    assert(!(qf - qr) < 1e-6);
    if (qs[0] > 0.0)
        qs = -qs;
    std::cout << "CC:C>S: Setup " << setup << " is " << (acos(qs[3]) * 180.0 * 2.0 / M_PI) << std::endl;
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
    std::cout << "Solving camera " << camera<< " from setup " << setup << " QR: " << qr << " QC: " << qc << " QS: " << qs << " | " << " CR: " << cr << " CC: " << cc << " CS: " << cs << std::endl;
    assert(!(cr - cf) < 1e-6);
    assert(!(qf - qr) < 1e-6);
    if (qc[0] > 0.0)
        qc = -qc;
    std::cout << "CC:S>C: Camera " << camera << " is " << (acos(qc[3]) * 180.0 * 2.0 / M_PI) << std::endl;
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
            if (max_diff < !(v2 - v1))
                max_diff = !(v2 - v1);
        }
    }
    std::cout << "CCMXDIFF: " << max_diff << std::endl;
}

double PhotoStationCalibrator::trySolveInitialLocations(std::vector<int> &order)
{
    using std::vector;
    using std::queue;
    using corecvs::Vector3dd;
    using corecvs::Quaternion;

    vector<bool> camsSolved(N);
    vector<bool> setupsSolved(N);
    queue<int> cameraQueue;

    int id1 = order[0];
    camsSolved[id1] = true;
    cameraQueue.push(id1);
    relativeCameraPositions[id1].extrinsics = CameraLocationData(Vector3dd(0.0, 0.0, -120.0), Quaternion(0.0, 0.0, 0.0, 1.0));
    for (int i = 0; i < M; ++i)
    {
        if (!initialGuess[i][id1].first || setupsSolved[i]) continue;
        solveCameraToSetup(initialGuess[i][id1].second, id1, i);
        setupsSolved[i] = true;
    }

    double retVal = 1e100;

    for (int i = 1; i < N; ++i)
    {
        int id = order[i];
        bool solved = false;
        for (int j = 0; j < M; ++j)
            if (initialGuess[j][id].first && setupsSolved[j])
            {
                solveSetupToCamera(initialGuess[j][id].second, id, j);
                camsSolved[id] = true;
                solved = true;
                break;
            }
        if (!solved) return retVal;
        for (int j = 0; j < M; ++j)
        {
            if (!initialGuess[j][id].first || setupsSolved[j]) continue;
            solveCameraToSetup(initialGuess[j][id].second, id, j);
            setupsSolved[j] = true;
        }
    }
    for (int i = 0; i < N; ++i)
        if (!camsSolved[i])
            return retVal;

    double max_diff = 0;
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
            if (max_diff < !(v2 - v1))
                max_diff = !(v2 - v1);
        }
    }

    return max_diff;
}

void PhotoStationCalibrator::solveInitialLocations()
{
    // 1. fix first' camera internal position and orientation to (0,0,0), (0,0,0,1)
    // 2. initialize all uninitialized setups that has that camera
    // 3. select uninitalized camera reachable from initialized setups and goto 2
    // 4. check if all setups were initialized
    // 5. Provide info (e.g. should we run LM, or it's useless)
    //
    decltype(relativeCameraPositions) bestPos;
    decltype(absoluteSetupLocation) bestLocs;
    double bestMax = 1e10;
#if 1
    std::vector<int> order(N);;
    for (int i = 0; i < N; ++i)
        order[i] = i;
    do
    {
        double res = trySolveInitialLocations(order);
        if (res < bestMax)
        {
            bestMax = res;
            bestPos = relativeCameraPositions;
            bestLocs = absoluteSetupLocation;
        }
    } while (std::next_permutation(order.begin(), order.end()));
    relativeCameraPositions = bestPos;
    absoluteSetupLocation = bestLocs;
#else
    for (int id1 = 0; id1 < 6; ++id1)
    {
    std::vector<bool> camsSolved(N);
    std::vector<bool> setupsSolved(N);
    std::queue<int> cameraQueue;

    camsSolved[id1] = true;
    cameraQueue.push(id1);
    relativeCameraPositions[id1].extrinsics = { corecvs::Vector3dd(0.0, 0.0, -120.0), corecvs::Quaternion(0.0, 0.0, 0.0, 1.0)};

    corecvs::Quaternion total(0.0, 0.0, 0.0, 1.0);
    std::cout << "CC" << std::endl << "CC" << std::endl;
    for (int j = 0; j < M; ++j)
    {
        std::cout << "CC|PS: " << j << std::endl;
        corecvs::Quaternion q1 = corecvs::Quaternion(0.0, 0.0, 0.0, 1.0);
        int curr = 0;
        int prev = -1;
        for (int i = 0; i < N; ++i)
        {
            if (!initialGuess[j][i].first)
                continue;
            corecvs::Quaternion q = (initialGuess[j][i].second.orientation ^ q1.conjugated());
            corecvs::Vector3dd axis(q[0], q[1], q[2]);
            axis.normalise();
            double angle = acos(q[3]) * 2.0 * 180.0 / M_PI;
            std::cout << "CC|PSC" << i << ": " << initialGuess[j][i].second.position << " " << initialGuess[j][i].second.orientation << " " <<  axis << " ( " << angle << " ) " << std::endl;
            q1 = initialGuess[j][i].second.orientation;
            curr ++;
            if (curr == 2){
            if (prev + 1 == i)
                total = total ^ q;
            else
                total = total ^ q.conjugated();
            std::cout << "CC|QU" << total << " " << (acos(total[3]) * 180.0 * 2.0 / M_PI) << std::endl;
            }
            prev = i;
        }
    }
    std::cout << "CC|TOTAL: " << total << std::endl;
    while (cameraQueue.size())
    {
        int cam = cameraQueue.front(); cameraQueue.pop();

        for (int setup = 0; setup < M; ++setup)
        {
//            validate();
            for (int ccm = 0; ccm < N; ++ccm)
            {
                if (!initialGuess[setup][ccm].first || !camsSolved[ccm] || !setupsSolved[setup])
                    continue;
                corecvs::Vector3dd  cc = relativeCameraPositions[ccm].extrinsics.position;
                corecvs::Quaternion qc = relativeCameraPositions[ccm].extrinsics.orientation;
                corecvs::Vector3dd  cr = initialGuess[setup][ccm].second.position;
                corecvs::Quaternion qr = initialGuess[setup][ccm].second.orientation;
                corecvs::Vector3dd  cs = absoluteSetupLocation[setup].position;
                corecvs::Quaternion qs = absoluteSetupLocation[setup].orientation;

                std::cout << "Solving: Checking camera " << ccm << " from setup " << setup << std::endl;
                std::cout << "Solving setup " << setup << " from camera " << ccm << " QR: " << qr << " QC: " << qc << " QS: " << qs << " | " << " CR: " << cr << " CC: " << cc << " CS: " << cs << std::endl;
                corecvs::Quaternion qq = (qc ^ qs) ^ qr.conjugated();
                std::cout << "CC:QQ" << qq << std::endl;
                double diffa = 2.0 * qq[3] * qq[3] - 1.0;
//                assert (acos(diffa) < 15.0 * M_PI / 180.0);
                assert ((cs + qs.conjugated() * cc) == (cs + (qs.conjugated() * cc)));
                if(!(cr - (cs + qs.conjugated() * cc)) >= 60.0)
                {
                    validate();
                }
//                assert (!(cr - (cs + qs.conjugated() * cc)) < 60.0);
            }
            if (!initialGuess[setup][cam].first || setupsSolved[setup])
                continue;

            std::cout << "Solving setup " << setup << " from cam " << cam << std::endl;
            solveCameraToSetup(initialGuess[setup][cam].second, cam, setup);
            setupsSolved[setup] = true;

            for (int camN = 0; camN < N; ++camN)
            {
                if (!initialGuess[setup][camN].first || camsSolved[camN])
                    continue;

                std::cout << "Solving camera " << camN << " from setup " << setup << std::endl;
                solveSetupToCamera(initialGuess[setup][camN].second, camN, setup);
                camsSolved[camN] = true;
                cameraQueue.push(camN);
            }

        }
    }


    bool ok = true;
    for (int cam = 0; cam < N; ++cam)
    {
        if (!camsSolved[cam])
        {
            std::cout << "Failed to solve cam " << cam << std::endl;
            ok = false;
        }
    }

    for (int set = 0; set < M; ++set)
    {
        if (!setupsSolved[set])
        {
            std::cout << "Failed to solve setup " << set << std::endl;
            ok = false;
        }
    }

    std::cout << "Initial validation" << std::endl;
    validate();
    for (int c1 = 0; c1 < N; ++c1)
    {
        for (int c2 = c1 + 1; c2 < N; ++c2)
        {
            for (int s = 0; s < M; ++s)
            {
                if (!initialGuess[s][c1].first || !initialGuess[s][c2].first)
                    continue;

                auto dd1 = initialGuess[s][c1].second.orientation.conjugated() ^ initialGuess[s][c2].second.orientation;
                auto dd2 = relativeCameraPositions[c1].extrinsics.orientation.conjugated() ^ relativeCameraPositions[c2].extrinsics.orientation;
                std::cout << "xxxdiff " << c1 << "-" << c2 << " : " << dd1 << " , " << dd2 << std::endl;
            }
        }
    }
    std::cout << "xxx" << std::endl;
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
            if (max_diff < !(v2 - v1))
                max_diff = !(v2 - v1);
        }
    }
#endif
#if 0
    std::cout << "CCMXDIFF: " << max_diff << std::endl;
    if (max_diff < bestMax)
    {
        bestLocs = absoluteSetupLocation;
        bestPos = relativeCameraPositions;
        bestMax = max_diff;
    }
    }
#endif
    std::cout << "FINALE: " << bestMax << std::endl;
    absoluteSetupLocation = bestLocs;
    relativeCameraPositions = bestPos;
    corecvs::Quaternion prev(0.0, 0.0, 0.0, 1.0);
    for (int cam = 0; cam < N; ++cam){
    for (int set = 0; set < M; ++set)
    {
        if (!initialGuess[set][cam].first)
            continue;
        auto diff = initialGuess[set][cam].second.orientation;
//        prev = absoluteSetupLocation[set].orientation;
        if (diff[0] > 0.0)
            diff = -diff;
        auto ang = acos(diff.t()) * 2.0 * 180.0 / M_PI;
        diff /= std::sqrt(1.0 - diff.t() * diff.t());

        std::cout << "Adiff: " <<  diff << " " << ang << std::endl;
    }
     std::cout << std::endl;
    }

    // It's useless in terms of RMSE
#if 0
    std::cout << std::endl;
    refineStruct();
#endif
}
