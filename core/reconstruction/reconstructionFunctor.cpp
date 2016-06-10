#include "reconstructionFunctor.h"

#include <set>

corecvs::ReconstructionFunctor::ReconstructionFunctor(corecvs::ReconstructionFixtureScene *scene, const std::vector<CameraFixture*> &optimizableSubset, const ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType &error, const corecvs::ReconstructionFunctorOptimizationType &optimization, bool excessiveQuaternionParametrization, const double pointErrorEstimate) : corecvs::SparseFunctionArgs(), scene(scene), error(error), optimization(optimization), excessiveQuaternionParametrization(excessiveQuaternionParametrization), scalerPoints(pointErrorEstimate), optimizableSubset(optimizableSubset)
{
    /*
     * Compute inputs (these are ball-park estimate, need to
     * re-calculate real DoFs later):
     *
     * A) orientable fixtures       (>= 2 3d-points)
     * B) translatable fixtures     (>= 3 3d-points)
     *
     * C) focal-tunable cameras     (>= 4 3d-points)
     * D) principal-tunable cameras (>= 6 3d-points)
     */
    computeInputs();
    /*
     * Compute outputs:
     * A) Reconstructed point errors
     * B) Static point errors
     * C) Errors in position-estimates
     */
    computeOutputs();
    // compute dependency
    computeDependency();
    // re-create base (weird, but we do not know input/output/dependency before creation)
    init(getInputNum(), getOutputNum(), sparsity);
}

struct PointFunctor: corecvs::FunctionArgs
{
    PointFunctor(corecvs::SceneFeaturePoint *pt, ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType errType, int N)
        : corecvs::FunctionArgs(3, (int)pt->observations__.size() * N),
          pt(pt),
          errType(errType),
          N(N)
    {
        for (auto& o: pt->observations__)
        {
            auto cam = o.second.cameraFixture->getWorldCamera(o.second.camera);
            auto mat = cam.getCameraMatrix();
            auto   R = cam.extrinsics.orientation.toMatrix();
            cached.emplace_back(std::move(cam), mat, R, o.second.observation);
        }
    }
    void operator() (const double* in, double* out)
    {
       auto vec = pt->reprojectedPosition = corecvs::Vector3dd(in[0], in[1], in[2]);
       int argout = 0;
#define EC(E, EE, EEE) \
        case ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType::E: \
            for (auto& o: cached) \
            { \
                auto  e = std::get<0>(o).EE(vec, std::get<3>(o)); \
                for (int j = 0; j < N; ++j) \
                    out[argout++] = EEE; \
            } \
            break;
        switch(errType)
        {
            EC(REPROJECTION,  reprojectionError, e[j])
            EC(ANGULAR,       angleError,        e)
            EC(CROSS_PRODUCT, crossProductError, e[j])
            EC(RAY_DIFF,      rayDiffError,      e[j])
            default:
                CORE_ASSERT_TRUE_S(false);
                break;
        }
    }
#undef EC

    corecvs::Matrix jacobianReprojection(const double* in)
    {
        int M = cached.size();
        corecvs::Matrix J(M * N, 3);
        corecvs::Vector3dd X(in[0], in[1], in[2]);
        int argout = 0;
        for (int i = 0; i < M; ++i)
        {
            auto P = std::get<1>(cached[i]);
            auto U = P * X;
            const double &ux = U[0], &uy = U[1], &uz = U[2];
            const double &p11 = P.a(0, 0), &p12 = P.a(0, 1), &p13 = P.a(0, 2),
                         &p21 = P.a(1, 0), &p22 = P.a(1, 1), &p23 = P.a(1, 2),
                         &p31 = P.a(2, 0), &p32 = P.a(2, 1), &p33 = P.a(2, 2);
            double e11 = 1.0 / uz,                 e13 = -ux / uz / uz,
                                   e22 = 1.0 / uz, e23 = -uy / uz / uz;
            J.a(argout, 0) = e11 * p11 + e13 * p31; J.a(argout, 1) = e11 * p12 + e13 * p32; J.a(argout, 2) = e11 * p13 + e13 * p33;
            ++argout;
            J.a(argout, 0) = e22 * p21 + e23 * p31; J.a(argout, 1) = e22 * p22 + e23 * p32; J.a(argout, 2) = e22 * p23 + e23 * p33;
            ++argout;
        }
        return J;
    }

    corecvs::Matrix jacobianRayDiff(const double* in)
    {
        int M = cached.size();
        corecvs::Matrix J(M * N, 3);
        corecvs::Vector3dd X(in[0], in[1], in[2]);
        int argout = 0;
        for (int i = 0; i < M; ++i)
        {
            auto R = std::get<2>(cached[i]);
            auto U = R * X;
            const double &x = U[0], &y = U[1], &z = U[2];
            double n = std::sqrt(U & U);
            double n3= n * n * n,
                   x2 = x * x,
                   y2 = y * y,
                   z2 = z * z,
                   xy = x * y,
                   xz = x * z,
                   yz = y * z;
            corecvs::Matrix33 D(
                    1.0 / n - x2 / n3,          -xy / n3,          -xz / n3,
                             -xy / n3, 1.0 / n - y2 / n3,          -yz / n3,
                             -xz / n3,          -yz / n3, 1.0 / n - z2 / n3);
            auto JJ = D * J;
            for (int ii = 0; ii < 3; ++ii)
                for (int jj = 0; jj < 3; ++jj)
                    J.a(argout + ii, jj) = D.a(ii, jj);
            argout += 3;
        }
        return J;
    }
    corecvs::Matrix getJacobian(const double* in, double delta = 1e-7)
    {
        switch(errType)
        {
            case ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType::ANGULAR:
            case ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType::CROSS_PRODUCT:
                return corecvs::FunctionArgs::getJacobian(in, delta);
            case ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType::REPROJECTION:
                return jacobianReprojection(in);
            case ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType::RAY_DIFF:
                return jacobianRayDiff(in);
            default:
                CORE_ASSERT_TRUE_S(false);
                return corecvs::Matrix(-1, -1);
                break;
        }
    }
    corecvs::SceneFeaturePoint* pt;
    ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType errType;

    std::vector<std::tuple<corecvs::FixtureCamera, corecvs::Matrix44, corecvs::Matrix33, corecvs::Vector2dd>> cached;
    int N;
};

void corecvs::ReconstructionFunctor::alternatingMinimization(int steps)
{
    int N = (int)scene->trackedFeatures.size();
    int ec = getErrorComponentsPerPoint();
    int bs = N / 256;
    corecvs::parallelable_for(0, N, bs, [&](const corecvs::BlockedRange<int> &r)
        {
            for (int i = r.begin(); i < r.end(); ++i)
            {
                auto& pt = *scene->trackedFeatures[i];
                int /*in = 3,*/ out = (int)pt.observations__.size() * ec;
                PointFunctor pf(&pt, error, ec);
                corecvs::LevenbergMarquardt lm(steps);
                lm.f = &pf;
                lm.traceProgress = false;
                std::vector<double> inn(&pt.reprojectedPosition[0], &pt.reprojectedPosition[3]), outt(out);
                auto res = lm.fit(inn, outt);
                for (int ii = 0; ii < 3; ++ii)
                    pt.reprojectedPosition[ii] = res[ii];
            }
        });
}

int corecvs::ReconstructionFunctor::getInputNum()
{
    return
        (int)orientableFixtures.size()     * (excessiveQuaternionParametrization ? INPUTS_PER_ORIENTATION_EXC : INPUTS_PER_ORIENTATION_NEX)
      + (int)translateableFixtures.size()  * INPUTS_PER_TRANSLATION
      + (int)focalTunableCameras.size()    * INPUTS_PER_FOCAL
      + (int)principalTunableCameras.size()* INPUTS_PER_PRINCIPAL
      + (int)scene->trackedFeatures.size() * INPUTS_PER_3D_POINT;
}

int corecvs::ReconstructionFunctor::getOutputNum()
{
    return lastProjection + (int)positionConstrainedCameras.size() * OUTPUTS_PER_POSITION_CONSTRAINT;
}

#define IFNOT(cond, expr) \
    if (!(optimization & ReconstructionFunctorOptimizationType::cond)) \
    { \
        expr; \
    }
#define IF(cond, expr) \
    if (!!(optimization & ReconstructionFunctorOptimizationType::cond)) \
    { \
        expr; \
    }
#define GETPARAM(ref) \
    ref = in[argin++];
#define IF_GETPARAM(cond, ref) \
    if (!!(optimization & ReconstructionFunctorOptimizationType::cond)) ref = in[argin++];
#define IFNOT_GETPARAM(cond, ref) \
    if ( !(optimization & ReconstructionFunctorOptimizationType::cond)) ref = in[argin++];
#define SETPARAM(ref) \
    out[argout++] = ref;
#define IF_SETPARAM(cond, ref) \
    if (!!(optimization & ReconstructionFunctorOptimizationType::cond)) out[argout++] = ref;
#define IFNOT_SETPARAM(cond, ref) \
    if ( !(optimization & ReconstructionFunctorOptimizationType::cond)) out[argout++] = ref;

void corecvs::ReconstructionFunctor::computePointCounts()
{
    counter.clear();
    for (auto& pt: scene->trackedFeatures)
        for (auto& obs: pt->observations__)
        {
            if (!std::contains(optimizableSubset, obs.first.u))
                continue;
            counter[obs.first.u]++;
            counter[obs.first.v]++;
        }

    for (auto& pt: scene->staticPoints)
        for (auto& obs: pt->observations__)
        {
            if (!std::contains(optimizableSubset, obs.first.u))
                continue;
            counter[obs.first.u]++;
            counter[obs.first.v]++;
        }
}


void corecvs::ReconstructionFunctor::computeInputs()
{
    orientableFixtures.clear();
    translateableFixtures.clear();
    focalTunableCameras.clear();
    principalTunableCameras.clear();

    computePointCounts();
    CORE_ASSERT_TRUE_S(scene->placedFixtures.size());
    IF(DEGENERATE_ORIENTATIONS,
        if (counter[scene->placedFixtures[0]] >= MINIMAL_TRACKED_FOR_ORIENTATION)
        {
            orientableFixtures.push_back(scene->placedFixtures[0]);
            std::cout << "DEGENERATE_ORIENTATIONS" << std::endl;
        })
    IF(NON_DEGENERATE_ORIENTATIONS,
        for (size_t i = 1; i < scene->placedFixtures.size(); ++i)
            if (counter[scene->placedFixtures[i]] >= MINIMAL_TRACKED_FOR_ORIENTATION)
                orientableFixtures.push_back(scene->placedFixtures[i]);
        std::cout << "NON_DEGENERATE_ORIENTATIONS: " << orientableFixtures.size() << std::endl;)

    IF(DEGENERATE_TRANSLATIONS,
        if (counter[scene->placedFixtures[0]] > MINIMAL_TRACKED_FOR_TRANSLATION)
        {
            translateableFixtures.push_back(scene->placedFixtures[0]);
            std::cout << "DEGENERATE_TRANSLATIONS" << std::endl;
        })

    IF(NON_DEGENERATE_TRANSLATIONS,
        for (size_t i = 1; i < scene->placedFixtures.size(); ++i)
            if (counter[scene->placedFixtures[i]] >= MINIMAL_TRACKED_FOR_TRANSLATION)
                translateableFixtures.push_back(scene->placedFixtures[i]);
        std::cout << "NON_DEGENERATE_TRANSLATIONS: " << translateableFixtures.size() << std::endl;)

    std::set<FixtureCamera*> distinctCameras;
    for (auto& fixture: scene->placedFixtures)
        for (auto& cam: fixture->cameras)
            distinctCameras.insert(cam);

    IF(FOCALS,
        for (auto& cam: distinctCameras)
            if (counter[cam] > MINIMAL_TRACKED_FOR_FOCALS)
                focalTunableCameras.push_back(cam);
        std::cout << "FOCALS: " << focalTunableCameras.size() << std::endl;)

    IF(PRINCIPALS,
        for (auto& cam: distinctCameras)
            if (counter[cam] > MINIMAL_TRACKED_FOR_PRINCIPALS)
                principalTunableCameras.push_back(cam);
        std::cout << "PRINCIPALS: " << focalTunableCameras.size() << std::endl;)
}

void corecvs::ReconstructionFunctor::computeOutputs()
{
    positionConstrainedCameras.clear();
    lastProjection = 0;
    // This function does nothing except saving number of projections
    // and cameras with position constraints

    for (auto& pt: scene->trackedFeatures)
        lastProjection += (int)pt->observations__.size();
    for (auto& pt: scene->staticPoints)
        lastProjection += (int)pt->observations__.size();

    lastProjection *= getErrorComponentsPerPoint();

    IF (TUNE_GPS,
        for (auto& cf: scene->placedFixtures)
            if (scene->initializationData[cf].enforcePosition)
                positionConstrainedCameras.push_back(cf);)

    for (auto& f: orientableFixtures)
        originalOrientations.push_back(f->location.rotor);
    scalerPosition = scalerPoints = 1.0;
}

int corecvs::ReconstructionFunctor::getErrorComponentsPerPoint()
{
    switch(error)
    {
        case ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType::ANGULAR:
            return 1;
        case ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType::REPROJECTION:
            return 2;
        case ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType::CROSS_PRODUCT:
            return 3;
        case ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType::RAY_DIFF:
            return 3;
    }
    CORE_ASSERT_TRUE_S(false);
}

void corecvs::ReconstructionFunctor::computeDependency()
{
    revDependency.clear();
    sparsity.clear();
    cameraCache.clear();
    cacheRef.clear();
    cacheOrigin.clear();

    auto errSize = getErrorComponentsPerPoint();
    int id = 0, argin = 0;

    revDependency.resize(lastProjection);
    sparsity.resize(getInputNum());
    std::unordered_map<WPP, int> cacheIdx;


#define ALL_FROM(V) \
    for (auto& t: V) \
        for (auto& o: t->observations__) \
            for (int k = 0; k < errSize; ++k) \
                revDependency[id++] = &o.second;
    ALL_FROM(scene->trackedFeatures)
    ALL_FROM(scene->staticPoints)

    CORE_ASSERT_TRUE_S(id == lastProjection);
    cacheRef.resize(lastProjection);

    for (int i = 0; i < lastProjection; ++i)
    {
        auto p = revDependency[i];
        WPP wpp = WPP(p->cameraFixture, p->camera);
        if (!cacheIdx.count(wpp))
        {
            cameraCache.resize(1 + (cacheIdx[wpp] = cameraCache.size()));
            cacheOrigin.push_back(wpp);
        }
        cacheRef[i] = cacheIdx[wpp];
    }

#define DEPS(V, N, CPROJ, CPOS) \
    for (auto& a: V) \
    {\
        for (int i = 0; i < N; ++i) \
        { \
            CORE_ASSERT_TRUE_S(argin < sparsity.size()); \
            for (int j = 0; j < lastProjection; ++j) \
            { \
                auto p = revDependency[j]; \
                if (CPROJ) \
                    sparsity[argin].push_back(j); \
            } \
            for (int j = 0; j < (int)positionConstrainedCameras.size(); ++j) \
            { \
                auto p = positionConstrainedCameras[j]; \
                if (CPOS) \
                    for (int k = 0; k < OUTPUTS_PER_POSITION_CONSTRAINT; ++k) \
                        sparsity[argin].push_back(lastProjection + OUTPUTS_PER_POSITION_CONSTRAINT * j + k); \
                (void)p; \
            } \
            ++argin; \
        } \
    }
    DEPS(orientableFixtures,      (excessiveQuaternionParametrization ? INPUTS_PER_ORIENTATION_EXC : INPUTS_PER_ORIENTATION_NEX), a == p->cameraFixture, false)
    DEPS(translateableFixtures,   INPUTS_PER_TRANSLATION, a == p->cameraFixture, a == p);
    DEPS(focalTunableCameras,     INPUTS_PER_FOCAL,       a == p->camera, false)
    DEPS(principalTunableCameras, INPUTS_PER_PRINCIPAL,   a == p->camera, false)
    CORE_ASSERT_TRUE_S(argin == getInputNum() - (!(optimization & ReconstructionFunctorOptimizationType::POINTS) ? 0 : INPUTS_PER_3D_POINT * (int)scene->trackedFeatures.size()));
    int argin_prepoint = argin;
    IF(POINTS,
    DEPS(scene->trackedFeatures,  INPUTS_PER_3D_POINT,    a == p->featurePoint, false))
    CORE_ASSERT_TRUE_S(argin == getInputNum());
    schurBlocks.clear();
    for (int i = argin_prepoint; i < argin; i += INPUTS_PER_3D_POINT)
        schurBlocks.push_back(i);
    schurBlocks.push_back(argin);
}

void corecvs::ReconstructionFunctor::readParams(const double* params)
{
    int argin = 0;
#define FILL(V, N, C, A, P) \
    for (auto& a: V) \
    { \
        P; \
        for (int i = 0; i < N; ++i) \
        { \
            double v = params[argin++]; \
            C; \
        } \
        A; \
    }

    if (!inputQuaternions.size() && !excessiveQuaternionParametrization)
        inputQuaternions.resize(orientableFixtures.size());

    if (excessiveQuaternionParametrization)
        FILL(orientableFixtures,  INPUTS_PER_ORIENTATION_EXC, a->location.rotor[i] = v, a->location.rotor.normalise(),)
    else
        FILL(orientableFixtures,  INPUTS_PER_ORIENTATION_NEX, inputQuaternions[id][i] = v; a->location.rotor[i] = v, a->location.rotor[3] = 1.0; a->location.rotor.normalise(); a->location.rotor = a->location.rotor ^ originalOrientations[id], int id = &a - &orientableFixtures[0])
    FILL(translateableFixtures,   INPUTS_PER_TRANSLATION,     a->location.shift[i] = v,,)
    FILL(focalTunableCameras,     INPUTS_PER_FOCAL,           a->intrinsics.focal = corecvs::Vector2dd(v, v),,)
    FILL(principalTunableCameras, INPUTS_PER_PRINCIPAL,       a->intrinsics.principal[i] = v,,)
    FILL(scene->trackedFeatures,  INPUTS_PER_3D_POINT,        a->reprojectedPosition[i] = v,,)
    for (auto& wpp: cacheOrigin)
        cameraCache[&wpp - &cacheOrigin[0]] = wpp.u->getWorldCamera(wpp.v);
}

void corecvs::ReconstructionFunctor::writeParams(double* params)
{
    int argin = 0;
#define WRITE(V, N, C, P) \
    for (auto& a: V) \
    { \
        P; \
        for (int i = 0; i < N; ++i) \
            params[argin++] = C; \
    }

    if (excessiveQuaternionParametrization)
        WRITE(orientableFixtures,  INPUTS_PER_ORIENTATION_EXC, a->location.rotor[i],)
    else
        WRITE(orientableFixtures,  INPUTS_PER_ORIENTATION_NEX, qq[i], auto qq = inputQuaternions.size() ? inputQuaternions[&a - &orientableFixtures[0]] : corecvs::Vector3dd(0, 0, 0))
    WRITE(translateableFixtures,   INPUTS_PER_TRANSLATION, a->location.shift[i],)
    WRITE(focalTunableCameras,     INPUTS_PER_FOCAL,       a->intrinsics.focal[0],)
    WRITE(principalTunableCameras, INPUTS_PER_PRINCIPAL,   a->intrinsics.principal[i],)
    WRITE(scene->trackedFeatures,  INPUTS_PER_3D_POINT,    a->reprojectedPosition[i],)
}

void corecvs::ReconstructionFunctor::computeErrors(double *out, const std::vector<int> &idxs)
{
    std::vector<int> projections, positions;
    projections.reserve(idxs.size());
    positions.reserve(idxs.size());

    for (auto& id: idxs)
        (id < lastProjection ? projections : positions).push_back(id);

    int currLastProjection = (int)projections.size();
    CORE_ASSERT_TRUE_S(currLastProjection % getErrorComponentsPerPoint() == 0);

    ParallelErrorComputator computator(this, projections, out);
    int N = currLastProjection / getErrorComponentsPerPoint();
    int bs = N / 256;
    corecvs::parallelable_for(0, N, bs, computator, true);

    int idx = currLastProjection;
    for (int i = 0; i < (int)positions.size(); i += 3)
    {
        auto ps = positionConstrainedCameras[i / 3];

        auto diff = ps->location.shift - scene->initializationData[ps].initData.shift;
        auto foo = scene->initializationData[ps].positioningAccuracy * diff * scalerPosition;
        for (int j = 0; j < 3; ++j)
            out[idx++] = foo[j];
    }
}

corecvs::SparseMatrix corecvs::ReconstructionFunctor::getNativeJacobian(const double* in, double delta)
{
    readParams(in);
    switch(error)
    {
        case ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType::REPROJECTION:
            return jacobianReprojection();
        case ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType::RAY_DIFF:
            return jacobianRayDiff();
        default:
            return corecvs::SparseFunctionArgs::getJacobian(in, delta);
    }
}

corecvs::SparseMatrix corecvs::ReconstructionFunctor::jacobianRayDiff()
{
    auto U = ..., V = ...;
    double x = U[0], y = U[1], z = U[2];
    double x2 = x*x, y2 = y*y, z2 = z*z, xy = x*y, yz = y*z, xz = x*z;
    double N2 = x2 + y2 + z2;
    double N  = std::sqrt(N2);
    double N3 = N2 * N;

    corecvs::Matrix44 Derr(-x2/N3+1.0/N,        -xy/N3,        -xz/N3, 0.0,
                                 -xy/N3, -y^2/N3+1.0/N,        -yz/N3, 0.0,
                                 -xz/N3,        -yz/N3, -z^2/N3+1.0/N, 0.0,
                                    0.0,           0.0,           0.0, 0.0);
    double cx, cy, f, u, v;
    double cxmu = cx - u, cymv = cy - v;
    double cxmu2= cxmu * cxmu, cymv = cymv * cymv, cxmucymv = cxmu * cymv;
    double f2 = f * f;
    double f3 = f2 * f;
    N = 1.0 + (cxmu2+cymv2) / f2;
    double Ns = std::sqrt(N), N2 = N * N;
    auto Ns3 = N * Ns;

    corecvs::Vector4dd ErrF (      -Ns*cxmu/N2/f2,     -Ns*cymv/N2/f2, -(cxmu2+cymv2)/f3/Ns3, 1.0),
                       ErrCx( Ns*(f2+cymv2)/f3/N2, -cxmu * cxmv/f3/Ns3,          cxmu/f2/Ns3, 1.0),
                       ErrCy(    -cxmu*cxmv/f3/Ns, Ns*(f2+cxmu2)/f3/N2,          cxmv/f2/Ns3, 1.0);



}

corecvs::SparseMatrix corecvs::ReconstructionFunctor::jacobianReprojection()
{
    // Raydiff jacobian:
    // left:
    // N = sqrt(x^2+y^2+z^2)
    // N2= x^2+y^2+z^2
    // N3= N2*N
    //     / -x^2/N3+1/N      -xy/N3      -xz/N3 0 \
    // Pl= |      -xy/N3 -y^2/N3+1/N      -yz/N3 0 |
    //     |      -xz/N3      -yz/N3 -z^2/N3+1/N 0 |
    //     \           0           0           0 0 /
    //
    // N = 1+((cx-u)^2+(cy-v)^2)/f^2
    // Ns = sqrt(N)
    // Prf =
    //         /              Ns*(cx-u)/N^2/f^2 \
    //       - |              Ns*(cy-v)/N^2/f^2 |
    //         \ ((cx-u)^2+(cy-v)^2)/f^3/(Ns^3) /
    // Prcx =
    //         / Ns*(f^2+(cy-v)^2)/f^3/N^2 \
    //         |   -(cx-u)*(cy-v)/f^3/Ns^3 |
    //         \           (cx-u)/f^2/Ns^3 /
    // Prcy =
    //         /  -(cx-u)*(cy-v)/f^3/Ns^3) \
    //         | Ns*(f^2+(cx-u)^2)/f^3/N^2 |
    //         \          (cy-v)/(f^2/Ns^3 /
    //
    //
    //Reprojection error jacobian
    //
    // Pr= / 1/z   0  -x/z^2 0 \
    //     |   0 1/z  -y/z^2 0 |
    //     |   0   0       0 0 |
    //     \   0   0       0 0 /
    // Kf= / 1 0 0 0 \
    //     | 0 1 0 0 |
    //     | 0 0 0 0 |
    //     \ 0 0 0 0 /
    // Kcx=/ 0 0 1 0 \
    //     | 0 0 0 0 |
    //     | 0 0 0 0 |
    //     \ 0 0 0 0 /
    // Kcy=/ 0 0 0 0 \
    //     | 0 0 1 0 |
    //     | 0 0 0 0 |
    //     \ 0 0 0 0 /
    //
    // Rotations:
    // A) excessive parametrization
    // R = / 1-2(qy^2+qz^2) 2(qx*qy-qw*qz) 2(qx*qz+qy*qw) 0 \
    //     | 2(qx*qy+qz*qw) 1-2(qx^2+qz^2) 2(qy*qz-qw*qx) 0 |
    //     | 2(qx*qz-qw*qy) 2(qy*qz+qw*qx) 1-2(qx^2+qy^2) 0 |
    //     \              0              0              0 1 /
    //
    // Rqx=/    0  2*qy  2*qz 0 \
    //     | 2*qy -4*qx -2*qw 0 |
    //     | 2*qz  2*qw -4*qx 0 |
    //     \    0     0     0 0 /
    //
    // Rqy=/-4*qy 2*qx  2*qw 0 \
    //     | 2*qx    0  2*qz 0 |
    //     |-2*qw 2*qz -4*qy 0 |
    //     \    0    0     0 0 /
    //
    // Rqz=/-4*qz, -2*qw, 2*qx 0 \
    //     | 2*qw, -4*qz, 2*qy 0 |
    //     | 2*qx,  2*qy,    0 0 |
    //     \    0,     0,    0 0 /
    //
    // Rqw=/    0, -2*qz,  2*qy 0 \
    //     |  2*qz,    0, -2*qx 0 |
    //     | -2*qy, 2*qx,     0 0 |
    //     \     0     0      0 0 /
    //
    // B) non-excessive
    // N = qx^2+qy^2+qz^2-1
    //      / 2qy^2*N+2qz^2*N+1    2N(-qx*qy+qz)     -2N(qx*qz+qy) 0 \
    // R =|     -2N(qx*qy+qz) 2qx^2*N+2qz^2N+1      2N(qx-qy*qz) 0 |
    //    |     2N(-qx*qz+qy)    -2N(qx+qy*qz) 2qx^2*N+2qy^2*N+1 0 |
    //    \                 0                0                 0 1 /
    // Rqx =
    // /                     4qx(qy^2+qz^2) -6qx^2*qy+4qx*qz-2qy^3-2qy*qz^2+2qy -6qx^2*qz-4qx*qy-2qy^2*qz-2qz^3+2qz 0 \
    // |-6qx^2*qy-4qx*qz-2qy^3-2qy*qz^2+2qy             4qx(2qx^2+qy^2+2qz^2-1)       6qx^2-4qx*qy*qz+2qy^2+2qz^2-2 0 |
    // |-6qx^2*qz+4qx*qy-2qy^2*qz-2qz^3+2qz      -6qx^2-4qx*qy*qz-2qy^2-2qz^2+2             4qx(2qx^2+2qy^2+qz^2-1) 0 |
    // \                                  0                                   0                                   0 0 /
    // Rqy =
    // /            4qy(qx^2+2qy^2+2qz^2-1) -2qx^3-6qx*qy^2-2qx*qz^2+2qx+4qy*qz      -2qx^2-4qx*qy*qz-6qy^2-2qz^2+2 0 \
    // |-2qx^3-6qx*qy^2-2qx*qz^2+2qx-4qy*qz                      4qy(qx^2+qz^2) -2qx^2*qz+4qx*qy-6qy^2*qz-2qz^3+2qz 0 |
    // |      2qx^2-4qx*qy*qz+6qy^2+2qz^2-2 -2qx^2*qz-4qx*qy-6qy^2*qz-2qz^3+2qz             4qy(2qx^2+2qy^2+qz^2-1) 0 |
    // \                                  0                                   0                                   0 0 /
    // Rqz =
    // /            4qz(qx^2+2qy^2+2qz^2-1)       2qx^2-4qx*qy*qz+2qy^2+6qz^2-2 -2qx^3-2qx*qy^2-6qx*qz^2+2qx-4qy*qz  0 \
    // |     -2qx^2-4qx*qy*qz-2qy^2-6qz^2+2             4qz(2qx^2+qy^2+2qz^2-1) -2qx^2*qy+4qx*qz-2qy^3-6qy*qz^2+2qy  0 |
    // |-2qx^3-2qx*qy^2-6qx*qz^2+2qx+4qy*qz -2qx^2*qy-4qx*qz-2qy^3-6qy*qz^2+2qy                      4qz(qx^2+qy^2)  0 |
    // \                                  0                                   0                                   0  0 /
    //
    // Translations
    // T =
    // / 1 0 0 tx \
    // | 0 1 0 ty |
    // | 0 0 1 tz |
    // \ 0 0 0  1 /
    //
    // Ttx =
    // / 0 0 0 1 \
    // | 0 0 0 0 |
    // | 0 0 0 0 |
    // \ 0 0 0 0 /
    //
    // Tty =
    // / 0 0 0 0 \
    // | 0 0 0 1 |
    // | 0 0 0 0 |
    // \ 0 0 0 0 /
    //
    // Ttz =
    // / 0 0 0 0 \
    // | 0 0 0 0 |
    // | 0 0 0 1 |
    // \ 0 0 0 0 /

    // u' = Derr * K * Rc * (Rp * (X - Cp) - Cc)
    corecvs::Vector4dd X, Cp, Cc;

    double x = U[0], y = U[1], z = U[2];
    double z2= z * z;
    corecvs::Matrix44 Derr(1.0 / z,     0.0, -x / z2, 0.0,
                               0.0, 1.0 / z, -y / z2, 0.0,
                               0.0,     0.0,     0.0, 0.0,
                               0.0,     0.0,     0.0, 0.0);
    corecvs::Matrix44 Kf(1.0, 0.0, 0.0, 0.0,
                         0.0, 1.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0),
                     Kcx(0.0, 0.0, 1.0, 0.0,
                          0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0),
                     Kcy(0.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 1.0, 0.0,
                         0.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0);
    corecvs::Matrix44 Rc(...),
                      Tc(...);
    double qx, qy, qz, qw;
    corecvs::Matrix44 Rp, Rqx, Rqy, Rqz, Rqw;
    double tx = X[0], ty = X[1], tz = X[2];
    corecvs::Matrix44 Tp(1.0, 0.0, 0.0, -tx,
                         0.0, 1.0, 0.0, -ty,
                         0.0, 0.0, 1.0, -tz,
                         0.0, 0.0, 0.0, 1.0),
                     Tpx(0.0, 0.0, 0.0,-1.0,
                          0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0),
                     Tpy(0.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0,-1.0,
                         0.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0),
                     Tpz(0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0,-1.0,
                          0.0, 0.0, 0.0, 0.0);
    auto TpX       = Tp * X;
    auto RpTpX     = Rp * TpX;
    auto TcRpTpX   = Tc * RpTpX;
    auto RcTcRpTpX = Rc * TcRpTpX;

    auto DerrKRcTc     = Derr * K * Rc * Tc;
    auto DerrKRcTcRp   = DerrKRcTc * Rp;
    auto DerrKRcTcRpTp = DerrKRcTcRp * Tp;

    auto df = Derr * Kf * RcTcRpTpX,
         dcx= Derr * Kcx* RcTcRpTpX,
         dcy= Derr * Kcy* RcTcRpTpX,
         dx = DerrKRcTcRpTp * Xx,
         dy = DerrKRcTcRpTp * Xy,
         dz = DerrKRcTcRpTp * Xz,
         dtx= DerrKRcTcRp * Tpx * X,
         dty= DerrKRcTcRp * Tpy * X,
         dtz= DerrKRcTcRp * Tpz * X,
         dqx= DerrKRcTc * Rqx* TpX,
         dqy= DerrKRcTc * Rqy* TpX,
         dqz= DerrKRcTc * Rqz* TpX,
         dqw= DerrKRcTc * Rqw* TpX;


}

void corecvs::ReconstructionFunctor::QuaternionDiff(double qx, double qy, double qz, double qw, bool excessive, corecvs::Matrix44 &Rqx, corecvs::Matrix44 &Rqy, corecvs::Matrix44 &Rqz, corecvs::Matrix44 &Rqw)
{
    if (excessive)
    {
        Rqx = corecvs::Matrix44(    0.0, 2.0*qy, 2.0*qz, 0.0,
                                   2.0*qy,-4.0*qx,-2.0*qw, 0.0,
                                   2.0*qz, 2.0*qw,-4.0*qx, 0.0,
                                      0.0,    0.0,    0.0, 0.0);
        Rqy = corecvs::Matrix44(-4.0*qy, 2.0*qx, 2.0*qw, 0.0,
                                   2.0*qx,    0.0, 2.0*qz, 0.0,
                                  -2.0*qw, 2.0*qz,-4.0*qy, 0.0,
                                      0.0,    0.0,    0.0, 0.0);
        Rqz = corecvs::Matrix44(-4.0*qz,-2.0*qw, 2.0*qx, 0.0,
                                   2.0*qw,-4.0*qz, 2.0*qy, 0.0,
                                   2.0*qx, 2.0*qy,    0.0, 0.0,
                                      0.0,    0.0,    0.0, 0.0);
        Rqw = corecvs::Matrix44(    0.0,-2.0*qz, 2.0*qy, 0.0,
                                   2.0*qz,    0.0,-2.0*qx, 0.0,
                                  -2.0*qy, 2.0*qx,    0.0, 0.0,
                                      0.0,    0.0,    0.0, 0.0);
    }
    else
    {
        double qx2 = qx * qx, qy2 = qy * qy, qz2 = qz * qz, qxqyqz = qx * qy * qz;
        double qxqy= qx * qy, qxqz = qx * qz, qyqz = qy * qz;
               qx3 = qx2 * qx, qy3 = qy2 * qy, qz3 = qz2 * qz, qx2qy = qx2 * qy, qx2qz = qx2 * qz, qxqy2 = qx * qy2, qy2qz = qy2 * qz, qxqz2 = qx * qz2, qyqz2 = qy * qz2;
        Rqx = corecvs::Matrix44(                            4.0*qx*(qy2+qz2),-6.0*qx2qy+4.0*qxqz-2.0*qy3-2.0*qyqz2+2.0*qy,-6.0*qx2qz-4.0*qxqy-2.0*qy2qz-2.0*qz3+2.0*qz, 0.0,
                                -6.0*qx2qy-4.0*qxqz-2.0*qy3-2.0*qyqz2+2.0*qy,            4.0*qx*(2.0*qx2+qy2+2.0*qz2-1.0),        6.0*qx2-4.0*qxqyqz+2.0*qy2+2.0*qz2-2, 0.0,
                                -6.0*qx2qz+4.0*qxqy-2.0*qy2qz-2.0*qz3+2.0*qz,     -6.0*qx2-4.0*qxqyqz-2.0*qy2-2.0*qz2+2.0,            4.0*qx*(2.0*qx2+2.0*qy2+qz2-1.0), 0.0,
                                                                         0.0,                                         0.0,                                         0.0, 0.0);

        Rqy = corecvs::Matrix44(            4.0*qy*(qx2+2.0*qy2+2.0*qz2-1.0),-2.0*qx3-6.0*qxqy2-2.0*qxqz2+2.0*qx+4.0*qyqz,     -2.0*qx2-4.0*qxqyqz-6.0*qy2-2.0*qz2+2.0, 0.0,
                                -2.0*qx3-6.0*qxqy2-2.0*qxqz2+2.0*qx-4.0*qyqz,                            4.0*qy*(qx2+qz2),-2.0*qx2qz+4.0*qxqy-6.0*qy2qz-2.0*qz3+2.0*qz, 0.0,
                                      2.0*qx2-4.0*qxqyqz+6.0*qy2+2.0*qz2-2.0,-2.0*qx2qz-4.0*qxqy-6.0*qy2qz-2.0*qz3+2.0*qz,            4.0*qy*(2.0*qx2+2.0*qy2+qz2-1.0), 0.0,
                                                                         0.0,                                         0.0,                                         0.0, 0.0);

        Rqz = corecvs::Matrix44(            4.0*qz*(qx2+2.0*qy2+2.0*qz2-1.0),      2.0*qx2-4.0*qxqyqz+2.0*qy2+6.0*qz2-2.0,-2.0*qx3-2.0*qxqy2-6.0*qxqz2+2.0*qx-4.0*qyqz, 0.0,
                                     -2.0*qx2-4.0*qxqyqz-2.0*qy2-6.0*qz2+2.0,            4.0*qz*(2.0*qx2+qy2+2.0*qz2-1.0),-2.0*qx2qy+4.0*qxqz-2.0*qy3-6.0*qyqz2+2.0*qy, 0.0,
                                -2.0*qx3-2.0*qxqy2-6.0*qxqz2+2.0*qx+4.0*qyqz,-2.0*qx2qy-4.0*qxqz-2.0*qy3-6.0*qyqz2+2.0*qy,                            4.0*qz*(qx2+qy2), 0.0,
                                                                         0.0,                                         0.0,                                         0.0, 0.0);

    }
}


void corecvs::ParallelErrorComputator::operator() (const corecvs::BlockedRange<int> &r) const
{
    auto& revDependency = functor->revDependency;
    auto& cameraCache = functor->cameraCache;
    auto& cacheRef = functor->cacheRef;
    int N = functor->getErrorComponentsPerPoint();
    double *out = output;
#define EC(E, EE, EEE) \
    case ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType::E: \
        for (int ii = r.begin(); ii < r.end(); ++ii) \
        { \
            int i = idxs[ii * N]; \
            auto& o = *revDependency[i]; \
            auto& c = cameraCache[cacheRef[i]]; \
            auto  e = c.EE(o.featurePoint->reprojectedPosition, o.observation); \
            for (int j = 0; j < N; ++j) \
                out[ii * N + j] = EEE; \
        } \
        break;
    switch(functor->error)
    {
        EC(REPROJECTION,  reprojectionError, e[j])
        EC(ANGULAR,       angleError,        e)
        EC(CROSS_PRODUCT, crossProductError, e[j])
        EC(RAY_DIFF,      rayDiffError,      e[j])
        default:
            CORE_ASSERT_TRUE_S(false);
            break;
    }
#undef EC
}
