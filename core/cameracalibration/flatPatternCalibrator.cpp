#include "flatPatternCalibrator.h"

FlatPatternCalibrator::FlatPatternCalibrator(const CameraConstraints constraints, const PinholeCameraIntrinsics lockParams, const LineDistortionEstimatorParameters distortionEstimatorParams, const double lockFactor) : factor(lockFactor), K(0), N(0), absoluteConic(6), intrinsics(lockParams), lockParams(lockParams), distortionEstimationParams(distortionEstimatorParams), constraints(constraints), forceZeroSkew(!!(constraints & CameraConstraints::ZERO_SKEW))
{
    distortionParams.mMapForward = true;
}

void FlatPatternCalibrator::addPattern(const PatternPoints3d &patternPoints, const CameraLocationData &position)
{
    ++N;
    locationData.push_back(position);
    points.push_back(patternPoints);
    K += patternPoints.size();
}

void FlatPatternCalibrator::solve(bool runPresolver, bool runLM, int LMiterations)
{
    if (runPresolver)
    {
        solveInitialIntrinsics();
        solveInitialExtrinsics();
        std::cout << std::endl << this << "RES Full projective: " << getRmseReprojectionError() << std::endl;
        enforceParams();
    }
    std::cout << std::endl << this << "RES Constrained init: " << getRmseReprojectionError() << std::endl;

    if(runLM) refineGuess(LMiterations);
    std::cout << std::endl <<  this << "RES LM: " << getRmseReprojectionError() << std::endl;
    std::cout << "OPTFAC: " << factor << std::endl;
}

PinholeCameraIntrinsics FlatPatternCalibrator::getIntrinsics()
{
    return intrinsics;
}

std::vector<CameraLocationData> FlatPatternCalibrator::getExtrinsics()
{
    return locationData;
}

LensDistortionModelParameters FlatPatternCalibrator::getDistortion()
{
    CORE_ASSERT_TRUE_S(!!(constraints & CameraConstraints::UNLOCK_DISTORTION));
    return distortionParams;
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
            auto pp = ptp.second;

            pp[1] *= factor;

            auto res = intrinsics.project(R * (pp - C));
            if (!!(constraints & CameraConstraints::UNLOCK_DISTORTION))
            {
                CORE_ASSERT_TRUE_S(distortionParams.mMapForward);
                res = distortionParams.mapForward(res);
            }
            auto diff = res - ptp.first;

            out[idx++] = diff[0];
            out[idx++] = diff[1];
        }
    }
#ifdef PENALIZE_QNORM
    CORE_ASSERT_TRUE_S(idx == getOutputNum() - N);
#else
    CORE_ASSERT_TRUE_S(idx == getOutputNum());
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
    input++;

    if (!!(constraints & CameraConstraints::UNLOCK_DISTORTION))
    {
        int polyDeg = distortionEstimationParams.mPolinomDegree;
        if (distortionEstimationParams.mEvenPowersOnly)
            polyDeg /= 2;
        input += polyDeg;
        if (distortionEstimationParams.mEstimateTangent)
            input += 2;
        if (distortionEstimationParams.mEstimateCenter)
            input += 2;
    }

    IFNOT(UNLOCK_YSCALE, input--);
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

    double f = (intrinsics.focal.x() + intrinsics.focal.y()) / 2.0;
    FORCE(EQUAL_FOCAL, focal.x(), f);
    FORCE(EQUAL_FOCAL, focal.y(), f);
    LOCK(LOCK_FOCAL, focal.x());
    LOCK(LOCK_FOCAL, focal.y());

    LOCK(LOCK_PRINCIPAL, principal.x());
    LOCK(LOCK_PRINCIPAL, principal.y());
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
        locationData[i] = CameraLocationData(C, orientation);
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
            intrinsics.focal = Vector2dd(f,f);
            IFNOT_GET_PARAM(EQUAL_FOCAL, intrinsics.focal.y()));
    IFNOT(LOCK_PRINCIPAL,
            GET_PARAM(intrinsics.principal.x());
            GET_PARAM(intrinsics.principal.y()));
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
        }
        locationData[i].orientation.normalise();
    }
    IF_GET_PARAM(UNLOCK_YSCALE, factor);
    if (!!(constraints & CameraConstraints::UNLOCK_DISTORTION))
    {
        distortionParams.mMapForward = true;
        int polyDeg = distortionEstimationParams.mPolinomDegree;
        distortionParams.mKoeff.resize(polyDeg);
        for (auto& k: distortionParams.mKoeff)
            k = 0.0;
        int degStart = 0, degIncrement = 1;
        if (distortionEstimationParams.mEvenPowersOnly)
        {
            polyDeg /= 2;
            degStart = 1;
            degIncrement = 2;
        }
        for (int i = 0; i < polyDeg; ++i, degStart += degIncrement)
        {
            GET_PARAM(distortionParams.mKoeff[degStart]);
        }
        if (distortionEstimationParams.mEstimateTangent)
        {
            GET_PARAM(distortionParams.mTangentialX);
            GET_PARAM(distortionParams.mTangentialY);
        }
        if (distortionEstimationParams.mEstimateCenter)
        {
            GET_PARAM(distortionParams.mPrincipalX);
            GET_PARAM(distortionParams.mPrincipalY);
        }

    }
    CORE_ASSERT_TRUE_S(argin == getInputNum());
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
            SET_PARAM(intrinsics.focal.x());
            IFNOT_SET_PARAM(EQUAL_FOCAL, intrinsics.focal.y()));
    IFNOT(LOCK_PRINCIPAL,
            SET_PARAM(intrinsics.principal.x());
            SET_PARAM(intrinsics.principal.y()));
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
    IF_SET_PARAM(UNLOCK_YSCALE, factor);
    if (!!(constraints & CameraConstraints::UNLOCK_DISTORTION))
    {
        distortionParams.mMapForward = true;
        int polyDeg = distortionEstimationParams.mPolinomDegree;
        distortionParams.mKoeff.resize(polyDeg);
        int degStart = 0, degIncrement = 1;
        if (distortionEstimationParams.mEvenPowersOnly)
        {
            polyDeg /= 2;
            degStart = 1;
            degIncrement = 2;
        }
        for (int i = 0; i < polyDeg; ++i, degStart += degIncrement)
        {
            SET_PARAM(distortionParams.mKoeff[degStart]);
        }
        if (distortionEstimationParams.mEstimateTangent)
        {
            SET_PARAM(distortionParams.mTangentialX);
            SET_PARAM(distortionParams.mTangentialY);
        }
        if (distortionEstimationParams.mEstimateCenter)
        {
            SET_PARAM(distortionParams.mPrincipalX);
            SET_PARAM(distortionParams.mPrincipalY);
        }

    }
    CORE_ASSERT_TRUE_S(argout == getInputNum());
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

void FlatPatternCalibrator::refineGuess(int LMiterations)
{
    std::vector<double> in(getInputNum()), out(getOutputNum());
    distortionParams.mPrincipalX = intrinsics.cx();
    distortionParams.mPrincipalY = intrinsics.cy();
    distortionParams.mNormalizingFocal = !intrinsics.principal;
    writeParams(&in[0]);

    LevenbergMarquardt levmar(LMiterations);
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

    intrinsics = PinholeCameraIntrinsics(fx, fy, cx, cy, skew);
    intrinsics.size = lockParams.size;
    intrinsics.distortedSize = lockParams.distortedSize;
}
