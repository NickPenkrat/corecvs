#include "flatPatternCalibrator.h"
#include <cassert>

FlatPatternCalibrator::FlatPatternCalibrator(const CameraConstraints constraints, const CameraIntrinsics_ lockParams) : K(0), N(0), absoluteConic(6), lockParams(lockParams), constraints(constraints), forceZeroSkew(!!(constraints & CameraConstraints::ZERO_SKEW))
{
}

void FlatPatternCalibrator::addPattern(const PatternPoints3d &patternPoints, const LocationData &position)
{
    ++N;
    locationData.push_back(position);
    points.push_back(patternPoints);
    K += patternPoints.size();
}

void FlatPatternCalibrator::solve(bool runPresolver, bool runLM)
{
    if (runPresolver)
    {
        solveInitialIntrinsics();
        solveInitialExtrinsics();
        std::cout << std::endl << this << "RES Full projective: " << getRmseReprojectionError() << std::endl;
        enforceParams();
    }
    std::cout << std::endl << this << "RES Constrained init: " << getRmseReprojectionError() << std::endl;

    if(runLM) refineGuess();
    std::cout << std::endl <<  this << "RES LM: " << getRmseReprojectionError() << std::endl;
}

CameraIntrinsics_ FlatPatternCalibrator::getIntrinsics()
{
    return intrinsics;
}

std::vector<LocationData> FlatPatternCalibrator::getExtrinsics()
{
    return locationData;
}

double FlatPatternCalibrator::getRmseReprojectionError()
{
    std::vector<double> err(K * 2);
    getFullReprojectionError(&err[0]);

    double sqs = 0.0;
    for (auto e : err) {
        sqs += e * e;
    }
    return sqrt(sqs / K);
}

void FlatPatternCalibrator::getFullReprojectionError(double out[])
{
    int idx = 0;

    for (size_t i = 0; i < N; ++i)
    {
        auto& R = locationData[i].orientation;
        R /= R.l2Metric();
        auto& C = locationData[i].position;
        auto& pt = points[i];
        for (auto& ptp: pt)
        {
            auto res = intrinsics.project(R * (ptp.second - C));
            auto diff = res - ptp.first;

            out[idx++] = diff[0];
            out[idx++] = diff[1];
        }
    }
#ifdef PENALIZE_QNORM
    assert(idx == getOutputNum() - N);
#else
    assert(idx == getOutputNum());
#endif
}

#define IFNOT(cond, expr) \
    if (!(constraints & CameraConstraints::cond)) \
    { \
        expr; \
    }

int FlatPatternCalibrator::getInputNum() const
{
    int input = 0;
    IFNOT(LOCK_FOCAL,
            input ++;
            IFNOT(EQUAL_FOCAL,
                input++));
    IFNOT(LOCK_PRINCIPAL, input += 2);
    IFNOT(LOCK_SKEW, IFNOT(ZERO_SKEW, input++));

    input += 7 * (int)N;
    return input;
}

int FlatPatternCalibrator::getOutputNum() const
{
#ifdef PENALIZE_QNORM
    return (int)K * 2 + (int)N;
#else
    return (int)K * 2;
#endif
}

void FlatPatternCalibrator::enforceParams()
{
#define FORCE(s, a, b) \
    if (!!(constraints & CameraConstraints::s)) intrinsics.a = b;
#define LOCK(s, a) \
    if (!!(constraints & CameraConstraints::s)) intrinsics.a = lockParams.a;

    FORCE(ZERO_SKEW, skew, 0.0);
    LOCK(LOCK_SKEW, skew);

    double f = (intrinsics.fx + intrinsics.fy) / 2.0;
    FORCE(EQUAL_FOCAL, fx, f);
    FORCE(EQUAL_FOCAL, fy, f);
    LOCK(LOCK_FOCAL, fx);
    LOCK(LOCK_FOCAL, fy);

    LOCK(LOCK_PRINCIPAL, cx);
    LOCK(LOCK_PRINCIPAL, cy);
#undef FORCE
#undef LOCK
}

void FlatPatternCalibrator::solveInitialIntrinsics()
{
    computeHomographies();
    computeAbsoluteConic();
    extractIntrinsics();
}

void FlatPatternCalibrator::solveInitialExtrinsics()
{
    int n = (int)homographies.size();

    auto A = (corecvs::Matrix33)intrinsics;
    auto Ai = A.inv();

    for (int i = 0; i < n; ++i)
    {
        auto H = homographies[i];
        corecvs::Vector3dd h1(H.a(0, 0), H.a(1, 0), H.a(2, 0));
        corecvs::Vector3dd h2(H.a(0, 1), H.a(1, 1), H.a(2, 1));
        corecvs::Vector3dd h3(H.a(0, 2), H.a(1, 2), H.a(2, 2));

        double lambda = (1.0 / !(Ai * h1) + 1.0 / !(Ai * h2)) / 2.0;

        auto T = lambda * Ai * h3;
        auto r1 = lambda * Ai * h1;
        auto r2 = lambda * Ai * h2;
        auto r3 = r1 ^ r2;

        corecvs::Matrix33 R(r1[0], r2[0], r3[0],
                r1[1], r2[1], r3[1],
                r1[2], r2[2], r3[2]), V;

        corecvs::Vector3dd W;
        corecvs::Matrix::svd(&R, &W, &V);

        corecvs::Matrix33 RO = R * V.transposed();
        auto C = -RO.transposed() * T;

        corecvs::Quaternion orientation = corecvs::Quaternion::FromMatrix(RO);
        locationData[i] = LocationData(C, orientation);
    }
}

void FlatPatternCalibrator::readParams(const double in[])
{
#define GET_PARAM(ref) \
    ref = in[argin++];
#define IF_GET_PARAM(cond, ref) \
    if (!!(constraints & CameraConstraints::cond)) ref = in[argin++];
#define IFNOT_GET_PARAM(cond, ref) \
    if (!(constraints & CameraConstraints::cond)) ref = in[argin++];

    int argin = 0;
    IFNOT(LOCK_FOCAL,
            double f;
            GET_PARAM(f);
            intrinsics.fx = intrinsics.fy = f;
            IFNOT_GET_PARAM(EQUAL_FOCAL, intrinsics.fy));
    IFNOT(LOCK_PRINCIPAL,
            GET_PARAM(intrinsics.cx);
            GET_PARAM(intrinsics.cy));
    IFNOT(LOCK_SKEW,
            IFNOT_GET_PARAM(ZERO_SKEW, intrinsics.skew));

    for (size_t i = 0; i < N; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            GET_PARAM(locationData[i].position[j]);
        }
        for (int j = 0; j < 4; ++j)
        {
            GET_PARAM(locationData[i].orientation[j]);
            locationData[i].orientation.normalise();
        }
    }
    assert(argin == getInputNum());
#undef GET_PARAM
#undef IF_GET_PARAM
#undef IF_NOT_GET_PARAM
}

#define SET_PARAM(ref) \
    out[argout++] = ref;
#define IF_SET_PARAM(cond, ref) \
    if (!!(constraints & CameraConstraints::cond)) out[argout++] = ref;
#define IFNOT_SET_PARAM(cond, ref) \
    if (!(constraints & CameraConstraints::cond)) out[argout++] = ref;

void FlatPatternCalibrator::writeParams(double out[])
{
    int argout = 0;
    IFNOT(LOCK_FOCAL,
            SET_PARAM(intrinsics.fx);
            IFNOT_SET_PARAM(EQUAL_FOCAL, intrinsics.fy));
    IFNOT(LOCK_PRINCIPAL,
            SET_PARAM(intrinsics.cx);
            SET_PARAM(intrinsics.cy));
    IFNOT(LOCK_SKEW,
            IFNOT_SET_PARAM(ZERO_SKEW, intrinsics.skew));

    for (size_t i = 0; i < N; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            SET_PARAM(locationData[i].position[j]);
        }
        for (int j = 0; j < 4; ++j)
        {
            SET_PARAM(locationData[i].orientation[j]);
        }
    }

    assert(argout == getInputNum());
}

#undef SET_PARAM
#undef IF_SET_PARAM
#undef IFNOT_SET_PARAM
#undef IFNOT

void FlatPatternCalibrator::LMCostFunction::operator() (const double in[], double out[])
{
    calibrator->readParams(in);
    calibrator->getFullReprojectionError(out);

#ifdef PENALIZE_QNORM
    for (size_t i = 0; i < calibrator->N; ++i)
    {
        out[2 * calibrator->K + i] = 1.0 - calibrator->locationData[i].orientation.sumAllElementsSq();
    }
#endif
}

void FlatPatternCalibrator::refineGuess()
{
    std::vector<double> in(getInputNum()), out(getOutputNum());
    writeParams(&in[0]);

    LevenbergMarquardt levmar(1000);
    levmar.f = new LMCostFunction(this);

    auto res = levmar.fit(in, out);
    readParams(&res[0]);
}

void FlatPatternCalibrator::computeHomographies()
{
    homographies.clear();
    for (auto& pts: points)
    {
        if (!pts.size()) continue;

        std::vector<Vector2dd> ptsI, ptsP;

        for (auto& ptp: pts)
        {
            ptsI.push_back(ptp.first);
            ptsP.push_back(Vector2dd(ptp.second[0], ptp.second[1]));

        }

        HomographyReconstructor p2i;
        for (size_t i = 0; i < ptsI.size(); ++i)
        {
            p2i.addPoint2PointConstraint(ptsP[i], ptsI[i]);
        }

        corecvs::Matrix33 A, B;
        p2i.normalisePoints(A, B);
        auto res = p2i.getBestHomographyLSE();
        res = p2i.getBestHomographyLM(res);
        res = B.inv() * res * A;

        homographies.push_back(res);
    }
}

void FlatPatternCalibrator::computeAbsoluteConic()
{
    absoluteConic = corecvs::Vector(6);

    int n = (int)homographies.size();
    int n_equ = n * 2;
    if (n < 3) forceZeroSkew = true;

    if (forceZeroSkew) ++n_equ;
  //int n_equ_actual = n_equ;
    if (n_equ < 6) n_equ = 6;

    corecvs::Matrix A(n_equ, 6);
    for (int i = 0; i < n_equ; ++i)
    {
        for (int j = 0; j < 6; ++j)
        {
            A.a(i, j) = 0.0;
        }
    }

    for (int j = 0; j < n; ++j)
    {
        corecvs::Vector v00(6), v01(6), v11(6);
        auto& H = homographies[j];

#define V(I,J) \
        v ## I ## J[0] = H.a(0, I) * H.a(0, J); \
        v ## I ## J[1] = H.a(0, I) * H.a(1, J) + H.a(1, I) * H.a(0, J); \
        v ## I ## J[2] = H.a(1, I) * H.a(1, J); \
        v ## I ## J[3] = H.a(2, I) * H.a(0, J) + H.a(0, I) * H.a(2, J); \
        v ## I ## J[4] = H.a(2, I) * H.a(1, J) + H.a(1, I) * H.a(2, J); \
        v ## I ## J[5] = H.a(2, I) * H.a(2, J);

        V(0, 0);
        V(0, 1);
        V(1, 1);
#undef V

        auto v1 = v01;
        auto v2 = v00 - v11;

        for (int k = 0; k < 6; ++k)
        {
            A.a(2 * j, k) = v1[k];
            A.a(2 * j + 1, k) = v2[k];
        }
    }

    if (forceZeroSkew)
    {
        for (int i = 0; i < 6; ++i)
        {
            A.a(2 * n, i) = i == 1 ? 1.0 : 0.0;
        }
    }

    corecvs::Matrix V(6, 6), W(1, 6);
    corecvs::Matrix::svd(&A, &W, &V);

    double min_singular = 1e100;
    int id = -1;

    for (int j = 0; j < 6; ++j)
    {
        if (min_singular > W.a(0, j))
        {
            min_singular = W.a(0, j);
            id = j;
        }
    }

    for (int j = 0; j < 6; ++j)
    {
        absoluteConic[j] = V.a(j, id);
    }
}

void FlatPatternCalibrator::extractIntrinsics()
{
    double b11, b12, b22, b13, b23, b33;
    b11 = absoluteConic[0];
    b12 = absoluteConic[1];
    b22 = absoluteConic[2];
    b13 = absoluteConic[3];
    b23 = absoluteConic[4];
    b33 = absoluteConic[5];

    double cx, cy, fx, fy, lambda, skew;
    cy = (b12 * b13 - b11 * b23) / (b11 * b22 - b12 * b12);
    lambda = b33 - (b13 * b13 + cy * (b12 * b13 - b11 * b23)) / b11;
    fx = sqrt(lambda / b11);
    fy = sqrt(lambda * b11 / (b11 * b22 - b12 * b12));
    skew = -b12 * fx * fx * fy / lambda;
    cx = skew * cy / fx - b13 * fx * fx / lambda;

    intrinsics = CameraIntrinsics_(fx, fy, cx, cy, skew);
}