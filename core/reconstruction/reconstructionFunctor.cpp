#include "reconstructionFunctor.h"

#include <set>

const int CORE_COUNT_LIMIT = 256;

#if 0
#define IFUSED(p, N, A) \
            if (list.p != DependencyList::UNUSED) \
            { \
                for (int ii = 0; ii < N; ++ii) \
                    CORE_ASSERT_TRUE_S(list[&list.p - list.begin() + ii] != DependencyList::UNUSED); \
                A \
            }
#else
#define IFUSED(p, N, A) \
            if (list.p != DependencyList::UNUSED) \
            { \
                A \
            }
#endif

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
        int M = (int)cached.size();
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
        int M = (int)cached.size();
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
        }
        CORE_ASSERT_TRUE_S(false);
        return corecvs::Matrix(-1, -1);
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
      + (!(optimization & ReconstructionFunctorOptimizationType::POINTS) ? 0 : INPUTS_PER_3D_POINT * (int)scene->trackedFeatures.size());
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
    denseDependency.clear();
    sparseDependency.clear();
    sparseRowptr.clear();
    sparseCol.clear();
    revDependency.clear();
    sparsity.clear();
    cameraCache.clear();
    cacheRef.clear();
    cacheOrigin.clear();

    auto errSize = getErrorComponentsPerPoint();
    int id = 0, argin = 0, nOut = getOutputNum(), nIn = getInputNum();

    revDependency.resize(lastProjection);
    sparsity.resize(nIn);
    denseDependency.resize(nOut);
    sparseDependency.resize(nOut);
    depCache.clear();
    std::unordered_map<WPP, int> cacheIdx;
    std::unordered_map<SceneFeaturePoint*, DependencyList> sfpDepCache;


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
            cameraCache.resize(1 + (cacheIdx[wpp] = (int)cameraCache.size()));
            cacheOrigin.push_back(wpp);
        }
        cacheRef[i] = cacheIdx[wpp];
    }
    cjp.resize(cameraCache.size());

#define DEPS(V, N, CPROJ, CPOS, CPROJDEP, CPOSDEP) \
    for (auto& a: V) \
    {\
        for (int i = 0; i < N; ++i) \
        { \
            CORE_ASSERT_TRUE_S(argin < (int)sparsity.size()); \
            for (int j = 0; j < lastProjection; ++j) \
            { \
                auto p = revDependency[j]; \
                if (CPROJ) \
                { \
                    sparsity[argin].push_back(j); \
                    CPROJDEP; \
                } \
            } \
            for (int j = 0; j < (int)positionConstrainedCameras.size(); ++j) \
            { \
                auto p = positionConstrainedCameras[j]; \
                if (CPOS) \
                { \
                    for (int k = 0; k < OUTPUTS_PER_POSITION_CONSTRAINT; ++k) \
                        sparsity[argin].push_back(lastProjection + OUTPUTS_PER_POSITION_CONSTRAINT * j + k); \
                    CPOSDEP \
                } \
                (void)p; \
            } \
            ++argin; \
        } \
    }
#define BASED(W, N) \
    (&depCache[W].N)[i] = argin;
#define BASEDC(N) \
    BASED(WPP(p->cameraFixture, p->camera), N)

    DEPS(orientableFixtures,
         (excessiveQuaternionParametrization ? INPUTS_PER_ORIENTATION_EXC : INPUTS_PER_ORIENTATION_NEX),
         a == p->cameraFixture,
         false,
         BASEDC(qx),)
    DEPS(translateableFixtures,
         INPUTS_PER_TRANSLATION,
         a == p->cameraFixture,
         a == p,
         BASEDC(tx),
         BASED(WPP(p, WPP::VWILDCARD), tx)
        );
    DEPS(focalTunableCameras,
         INPUTS_PER_FOCAL,
         a == p->camera,
         false,
         BASEDC(f),)
    DEPS(principalTunableCameras,
         INPUTS_PER_PRINCIPAL,
         a == p->camera,
         false,
         BASEDC(cx),)
    CORE_ASSERT_TRUE_S(argin == getInputNum() - (!(optimization & ReconstructionFunctorOptimizationType::POINTS) ? 0 : INPUTS_PER_3D_POINT * (int)scene->trackedFeatures.size()));
    int argin_prepoint = argin;
    IF(POINTS,
        DEPS(scene->trackedFeatures,  INPUTS_PER_3D_POINT,    a == p->featurePoint, false, (&sfpDepCache[a].x)[i] = argin;,))
    CORE_ASSERT_TRUE_S(argin == getInputNum());
    schurBlocks.clear();
    for (int i = argin_prepoint; i < argin; i += INPUTS_PER_3D_POINT)
        schurBlocks.push_back(i);
    schurBlocks.push_back(argin);

    sparseRowptr.resize(nOut + 1);
    for (int i = 0; i < nOut; ++i)
    {
        auto &list = denseDependency[i];
        auto &sparseList = sparseDependency[i];
        if (i < lastProjection)
        {
            auto& pt= revDependency[i];
            auto& p = sfpDepCache[pt->featurePoint];
            auto& f = depCache[WPP(pt->cameraFixture, pt->camera)];
            list |= p;
            list |= f;
        }
        else
        {
            auto fp= positionConstrainedCameras[(i - lastProjection) / OUTPUTS_PER_POSITION_CONSTRAINT];
            auto &f = depCache[WPP(fp, WPP::VWILDCARD)];
//            std::cout << f.nnz() << ":";
            list |= f;
        }
        std::vector<int> used;
        for (auto& v: list)
            if (v != DependencyList::UNUSED)
                used.push_back(v);
        std::set<int> usedU(used.begin(), used.end());
        CORE_ASSERT_TRUE_S(usedU.size() == used.size());
  //      std::cout << used.size() << "|";
        std::sort(used.begin(), used.end());
        sparseRowptr[i + 1] = sparseRowptr[i] + (int)used.size();
        for (auto& u: used)
        {
            int id = (int)sparseCol.size();
            sparseCol.push_back(u);
            for (int j = 0; j < list.size(); ++j)
                if (list[j] == u)
                    sparseList[j] = id;
        }
    }
    std::cout << std::endl;
}

void corecvs::ReconstructionFunctor::readParams(const double* params, bool prepareJac)
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
    IF(POINTS,
        FILL(scene->trackedFeatures,  INPUTS_PER_3D_POINT,        a->reprojectedPosition[i] = v,,))

    bool exc = excessiveQuaternionParametrization;
    corecvs::parallelable_for((size_t)0, cacheOrigin.size(), std::max(cacheOrigin.size() / 256, (size_t)1),
    [&](const corecvs::BlockedRange<size_t> &r){
    for (auto iii = r.begin(); iii < r.end(); ++iii)
    {
        auto& wpp = cacheOrigin[iii];
        auto id = &wpp - &cacheOrigin[0];
        cameraCache[id] = wpp.u->getWorldCamera(wpp.v);
        auto& jc = cjp[id];
        auto& list = depCache[wpp];
        auto
            f  = wpp.v->intrinsics.focal[0],
            cx = wpp.v->intrinsics.principal[0],
            cy = wpp.v->intrinsics.principal[1],
            cqx= wpp.v->extrinsics.orientation[0],
            cqy= wpp.v->extrinsics.orientation[1],
            cqz= wpp.v->extrinsics.orientation[2],
            cqw= wpp.v->extrinsics.orientation[3],
            ctx= wpp.v->extrinsics.position[0],
            cty= wpp.v->extrinsics.position[1],
            ctz= wpp.v->extrinsics.position[2],
            fqx= wpp.u->location.rotor[0],
            fqy= wpp.u->location.rotor[1],
            fqz= wpp.u->location.rotor[2],
            fqw= wpp.u->location.rotor[3],
            ftx= wpp.u->location.shift[0],
            fty= wpp.u->location.shift[1],
            ftz= wpp.u->location.shift[2];
            corecvs::Matrix44
                K(  f, 0.0,  cx, 0.0,
                  0.0,   f,  cy, 0.0,
                  0.0, 0.0, 1.0, 0.0,
                  0.0, 0.0, 0.0, 0.0),
                Ki(1.0/f,   0.0, -cx/f, 0.0,
                     0.0, 1.0/f, -cy/f, 0.0,
                     0.0,   0.0,   1.0, 0.0,
                     0.0,   0.0,   0.0, 0.0),
                CT = Rotation(cqx, cqy, cqz, cqw, QuaternionParametrization::FULL, false)*Translation(-ctx, -cty, -ctz),
                FR = Rotation(fqx, fqy, fqz, fqw, QuaternionParametrization::FULL, true),
                FT = Translation(-ftx, -fty, -ftz),
                FR0(1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0);

            IFUSED(qx, rotationParams,
                    fqx = params[list.qx];
                    fqy = params[list.qy];
                    fqz = params[list.qz];
                    if (exc)
                        fqw = params[list.qw];
                    else
                    {
                        fqw = 1e100;
                        int id = 0;
                        for (; orientableFixtures[id] != wpp.u; ++id);
                        CORE_ASSERT_TRUE_S(id < (int)orientableFixtures.size());
                        FR0 = corecvs::Matrix44(originalOrientations[id].conjugated().toMatrix());
                    }
                    FR = Rotation(fqx, fqy, fqz, fqw, exc ? QuaternionParametrization::FULL_NORMALIZED : QuaternionParametrization::NON_EXCESSIVE, true);
                  )

            // df
            double f2 = f*f;
            double f21 = 1.0/(f*f);
            corecvs::Matrix44 Kf(1.0, 0.0, 0.0, 0.0,
                                0.0, 1.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0);
            corecvs::Matrix44 Kif(-f21,  0.0, cx / f2, 0.0,
                                    0.0, -f21, cy / f2, 0.0,
                                    0.0,  0.0,     0.0, 0.0,
                                    0.0,  0.0,     0.0, 0.0);
            // cx, cy
            corecvs::Matrix44 Kcx(0.0, 0.0, 1.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0);
            corecvs::Matrix44 Kicx(0.0, 0.0, -1.0/f, 0.0,
                                    0.0, 0.0,    0.0, 0.0,
                                    0.0, 0.0,    0.0, 0.0,
                                    0.0, 0.0,    0.0, 0.0);
            corecvs::Matrix44 Kcy(0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 1.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0);
            corecvs::Matrix44 Kicy(0.0, 0.0,    0.0, 0.0,
                                        0.0, 0.0, -1.0/f, 0.0,
                                    0.0, 0.0,    0.0, 0.0,
                                    0.0, 0.0,    0.0, 0.0);
            const corecvs::Matrix44
                    FTx(0.0, 0.0, 0.0,-1.0,
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0),
                    FTy(0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,-1.0,
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0),
                    FTz(0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,-1.0,
                        0.0, 0.0, 0.0, 0.0);
            corecvs::Vector4dd Xx(1.0, 0.0, 0.0, 0.0);
            corecvs::Vector4dd Xy(0.0, 1.0, 0.0, 0.0);
            corecvs::Vector4dd Xz(0.0, 0.0, 1.0, 0.0);
            corecvs::Matrix44 FRx, FRy, FRz, FRw;
            // qx, qy, qz, qw
            QuaternionDiff(fqx, fqy, fqz, fqw, exc ? QuaternionParametrization::FULL_NORMALIZED : QuaternionParametrization::NON_EXCESSIVE, true, FRx, FRy, FRz, FRw);
            jc.K   =   K ; jc.Ki =    Ki; jc.CT= CT; jc.FR= FR; jc.FT = FT; jc.FR0 = FR0;
            jc.Kf  =   Kf; jc.Kif=   Kif;
            jc.Kcx =  Kcx; jc.Kcy =  Kcy; jc.Kicx = Kicx; jc.Kicy = Kicy;
            jc.FRx = FRx; jc.FRy = FRy; jc.FRz  = FRz; jc.FRw  = FRw;

            auto CTFR0 = CT*FR0,
                 CTFR0FR = CT*FR0*FR;
            jc.CTFR0FRFT   =      CTFR0FR *FT;
            if (prepareJac)
            {
                jc.CTFR0FRxFT  =      CT*FR0*FRx*FT;
                jc.CTFR0FRyFT  =      CT*FR0*FRy*FT;
                jc.CTFR0FRzFT  =      CT*FR0*FRz*FT;
                jc.CTFR0FRwFT  =      CT*FR0*FRw*FT;
                jc.CTFR0FRFTx  =      CTFR0FR *FTx;
                jc.CTFR0FRFTy  =      CTFR0FR *FTy;
                jc.CTFR0FRFTz  =      CTFR0FR *FTz;
            }
            jc.KCTFR0FRFT   = K   * jc.CTFR0FRFT;
            if (prepareJac)
            {
                jc.KfCTFR0FRFT  = Kf  * jc.CTFR0FRFT;
                jc.KcxCTFR0FRFT = Kcx * jc.CTFR0FRFT;
                jc.KcyCTFR0FRFT = Kcy * jc.CTFR0FRFT;
                jc.KCTFR0FRxFT  = K   * jc.CTFR0FRxFT;
                jc.KCTFR0FRyFT  = K   * jc.CTFR0FRyFT;
                jc.KCTFR0FRzFT  = K   * jc.CTFR0FRzFT;
                jc.KCTFR0FRwFT  = K   * jc.CTFR0FRwFT;
                jc.KCTFR0FRFTx  = K   * jc.CTFR0FRFTx;
                jc.KCTFR0FRFTy  = K   * jc.CTFR0FRFTy;
                jc.KCTFR0FRFTz  = K   * jc.CTFR0FRFTz;
                jc.KCTFR0FRFTXx =      jc.KCTFR0FRFT * Xx;
                jc.KCTFR0FRFTXy =      jc.KCTFR0FRFT * Xy;
                jc.KCTFR0FRFTXz =      jc.KCTFR0FRFT * Xz;
                jc.CTFR0FRFTXx =      jc.CTFR0FRFT * Xx;
                jc.CTFR0FRFTXy =      jc.CTFR0FRFT * Xy;
                jc.CTFR0FRFTXz =      jc.CTFR0FRFT * Xz;
            }
    }});
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
    IF(POINTS,
        WRITE(scene->trackedFeatures,  INPUTS_PER_3D_POINT,    a->reprojectedPosition[i],))
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
    int bs = std::max(N / 256, 1);
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
    switch(error)
    {
        case ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType::REPROJECTION:
            return jacobianReprojection(in);
        case ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType::RAY_DIFF:
            return jacobianRayDiff(in);
        default:
            return corecvs::SparseFunctionArgs::getNativeJacobian(in, delta);
    }
}

corecvs::Matrix44 corecvs::ReconstructionFunctor::RayDiffJ(double ux, double uy, double uz)
{
    auto ux2 = ux*ux, uy2 = uy * uy, uz2 = uz * uz,
            uxuy= ux*uy, uxuz= ux * uz, uyuz= uy * uz;
    auto N = ux2 + uy2 + uz2;
    auto N2 = std::sqrt(N);
    auto N32 = N * N2, N21 = 1.0 / N2;
    return corecvs::Matrix44   (-ux2/N32+N21,    -uxuy/N32,    -uxuz/N32, 0.0,
                                    -uxuy/N32, -uy2/N32+N21,    -uyuz/N32, 0.0,
                                   -uxuz/N32,    -uyuz/N32, -uz2/N32+N21, 0.0,
                                         0.0,          0.0,          0.0, 0.0);
}

corecvs::SparseMatrix corecvs::ReconstructionFunctor::jacobianRayDiff(const double* in)
{
    readParams(in, true);
    std::vector<double> values(sparseCol.size());
    int nOut = getOutputNum(),
        lastProjection = (int)revDependency.size(),
        nErr = getErrorComponentsPerPoint();
    bool exc = excessiveQuaternionParametrization;
    int bs = std::max((lastProjection / nErr) / 256, 1);
    corecvs::parallelable_for(0, lastProjection / nErr, bs, [&](const corecvs::BlockedRange<int> &r){
    for (int ii= r.begin(); ii< r.end(); ++ii)
    {
        int i = ii * nErr;
        auto& list = sparseDependency[i];
        if (i < lastProjection)
        {
            /*
             * Here we prepare some useful things:
             * Rc, Rp, diffs, Ts, intrinsics
             */
#define SPARSE(p, ii) \
            values[sparseDependency[i + ii].p]
#define SPARSEV(p, v) \
            {SPARSE(p, 0) = v[0]; \
            SPARSE(p, 1) = v[1]; \
            SPARSE(p, 2) = v[2];}
#define SPARSEU(p) \
            {SPARSEV(p, dud ## p)}
            auto& o = *revDependency[i];
            auto pt = o.featurePoint;

            auto jc = cjp[cacheRef[i]];
            auto X3d = pt->reprojectedPosition;
            corecvs::Vector4dd X(X3d[0], X3d[1], X3d[2], 1.0);

            auto CTFR0FRFTX = jc.CTFR0FRFT*X;

            auto ux = CTFR0FRFTX[0], uy = CTFR0FRFTX[1], uz = CTFR0FRFTX[2];
            auto E = RayDiffJ(ux, uy, uz);
            auto ECTFR0FRFTX = E*CTFR0FRFTX;
            auto vx = o.observation[0], vy = o.observation[1], vz = 1.0;
            auto V = corecvs::Vector4dd(vx, vy, vz, 1.0);
            auto Vi = jc.Ki * V;
            auto E2= RayDiffJ(Vi[0], Vi[1], Vi[2]);

            IFUSED(f, INPUTS_PER_FOCAL,
                // df
                auto dudf = ECTFR0FRFTX - E2 * (jc.Kif * V);
                SPARSEU(f)
            )
            IFUSED(cx, INPUTS_PER_PRINCIPAL,
                // cx, cy
                auto dudcx = ECTFR0FRFTX - E2 * (jc.Kicx * V);
                auto dudcy = ECTFR0FRFTX - E2 * (jc.Kicy * V);
                SPARSEU(cx)
                SPARSEU(cy)
            )

            IFUSED(qx, rotationParams,
                // qx, qy, qz, qw
                auto dudqx = E * (jc.CTFR0FRxFT * X);
                auto dudqy = E * (jc.CTFR0FRyFT * X);
                auto dudqz = E * (jc.CTFR0FRzFT * X);
                auto dudqw = E * (jc.CTFR0FRwFT * X);
                SPARSEU(qx)
                SPARSEU(qy)
                SPARSEU(qz)
                if (exc)
                {
                    SPARSEU(qw)
                }
            )
            IFUSED(tx, INPUTS_PER_TRANSLATION,
                // tx, ty, tz
                auto dudtx = E * (jc.CTFR0FRFTx * X);
                auto dudty = E * (jc.CTFR0FRFTy * X);
                auto dudtz = E * (jc.CTFR0FRFTz * X);
                SPARSEU(tx)
                SPARSEU(ty)
                SPARSEU(tz)
            )
            IFUSED(x, INPUTS_PER_3D_POINT,
                // x, y, z
                auto dudx = E * jc.CTFR0FRFTXx;
                auto dudy = E * jc.CTFR0FRFTXy;
                auto dudz = E * jc.CTFR0FRFTXz;
                SPARSEU(x)
                SPARSEU(y)
                SPARSEU(z)
            )
        }
    }
    });
        for (int i = lastProjection; i < nOut;)
        {
            auto& list = sparseDependency[i];
            IFUSED(tx, 3,
                auto fixture = positionConstrainedCameras[(i - lastProjection) / OUTPUTS_PER_POSITION_CONSTRAINT];
                auto covariation = scene->initializationData[fixture].positioningAccuracy * scalerPosition;
                for (int ii = 0; ii < 3; ++ii)
                    for (int jj = 0; jj < 3; ++jj)
                        values[(&sparseDependency[i + ii].tx)[jj]] = covariation.a(ii, jj);
            )
            i += OUTPUTS_PER_POSITION_CONSTRAINT;
        }
    return SparseMatrix(getOutputNum(), getInputNum(), values, sparseCol, sparseRowptr);
}
#undef SPARSEV

corecvs::Matrix44 corecvs::ReconstructionFunctor::Rotation(double qx, double qy, double qz, double qw, corecvs::ReconstructionFunctor::QuaternionParametrization p, bool inverse)
{
    auto qx2 = qx * qx, qy2 = qy * qy, qz2 = qz * qz, qw2 = qw * qw,
         qxqy= qx * qy, qxqz= qx * qz, qxqw= qx * qw, qyqz= qy * qz, qyqw = qy * qw, qzqw = qz * qw;
    double N;
    corecvs::Matrix44 R;
    switch (p)
    {
        case QuaternionParametrization::FULL:
            R = corecvs::Matrix44(1.0 - 2.0*(qy2 + qz2),     2.0*(qxqy - qzqw),     2.0*(qxqz + qyqw), 0.0,
                                      2.0*(qxqy + qzqw), 1.0 - 2.0*(qx2 + qz2),     2.0*(qyqz - qxqw), 0.0,
                                      2.0*(qxqz - qyqw),     2.0*(qyqz + qxqw), 1.0 - 2.0*(qx2 + qy2), 0.0,
                                                    0.0,                   0.0,                   0.0, 1.0);
            break;
        case QuaternionParametrization::FULL_NORMALIZED:
            N = qx2 + qy2 + qz2 + qw2;
            R = corecvs::Matrix44((qw2+qx2-qy2-qz2),  2.0*(-qzqw+qxqy),   2.0*(qyqw+qxqz), 0.0,
                                    2.0*(qzqw+qxqy), (qw2-qx2+qy2-qz2),  2.0*(-qxqw+qyqz), 0.0,
                                   2.0*(-qyqw+qxqz),   2.0*(qxqw+qyqz), (qw2-qx2-qy2+qz2), 0.0,
                                                0.0,               0.0,               0.0,   N) / N;
            break;
        case QuaternionParametrization::NON_EXCESSIVE:
            N = (qx2 + qy2 + qz2 - 1.0) * 2.0;
            R = corecvs::Matrix44(N*qy2+N*qz2+1.0,    N*(-qxqy+qz),    -N*(qxqz+qy), 0.0,
                                     -N*(qxqy+qz), N*qx2+N*qz2+1.0,     N*(qx-qyqz), 0.0,
                                     N*(-qxqz+qy),    -N*(qx+qyqz), N*qx2+N*qy2+1.0, 0.0,
                                              0.0,             0.0,             0.0, 1.0);
            break;
    }
    return inverse ? R.transposed() : R;
}

corecvs::Matrix44 corecvs::ReconstructionFunctor::Translation(double tx, double ty, double tz)
{
    return corecvs::Matrix44(1.0, 0.0, 0.0, tx,
                             0.0, 1.0, 0.0, ty,
                             0.0, 0.0, 1.0, tz,
                             0.0, 0.0, 0.0, 1.0);
}

corecvs::SparseMatrix corecvs::ReconstructionFunctor::jacobianReprojection(const double* in)
{
    readParams(in, true);
    std::vector<double> values(sparseCol.size());
    int nOut = getOutputNum(),
        lastProjection = (int)revDependency.size(),
        nErr = getErrorComponentsPerPoint();
    bool exc = excessiveQuaternionParametrization;
    int bs = std::max((lastProjection / nErr) / 256, 1);
    corecvs::parallelable_for(0, lastProjection / nErr, bs, [&](const corecvs::BlockedRange<int> &r){
    for (int ii= r.begin(); ii< r.end(); ++ii)
    {
        int i = ii * nErr;
        auto& list = sparseDependency[i];
        if (i < lastProjection)
        {
#define SPARSE(p, ii) \
            values[sparseDependency[i + ii].p]
#define SPARSEV(p, v) \
            {SPARSE(p, 0) = v[0]; \
            SPARSE(p, 1) = v[1];}
#define SPARSEU(p) \
            {SPARSEV(p, dud ## p)}
            auto& o = *revDependency[i];
            auto pt = o.featurePoint;

            auto jc = cjp[cacheRef[i]];
            auto X3d = pt->reprojectedPosition;
            corecvs::Vector4dd X(X3d[0], X3d[1], X3d[2], 1.0);
            auto KCTFR0FRFTX= jc.KCTFR0FRFT * X;
            auto ux = KCTFR0FRFTX[0], uy = KCTFR0FRFTX[1], uz = KCTFR0FRFTX[2];
            corecvs::Matrix44 E(1.0 / uz,      0.0, -ux / uz / uz, 0.0,
                                     0.0, 1.0 / uz, -uy / uz / uz, 0.0,
                                     0.0,      0.0,           0.0, 0.0,
                                     0.0,      0.0,           0.0, 0.0);
            IFUSED(f, INPUTS_PER_FOCAL,
                // df
                auto dudf = E * (jc.KfCTFR0FRFT * X);
                SPARSEU(f)
            )
            IFUSED(cx, INPUTS_PER_PRINCIPAL,
                // cx, cy
                auto dudcx = E * (jc.KcxCTFR0FRFT * X);
                auto dudcy = E * (jc.KcyCTFR0FRFT * X);
                SPARSEU(cx)
                SPARSEU(cy)
            )

            IFUSED(qx, rotationParams,
                // qx, qy, qz, qw
                auto dudqx = E * (jc.KCTFR0FRxFT*X);
                auto dudqy = E * (jc.KCTFR0FRyFT*X);
                auto dudqz = E * (jc.KCTFR0FRzFT*X);
                auto dudqw = E * (jc.KCTFR0FRwFT*X);
                SPARSEU(qx)
                SPARSEU(qy)
                SPARSEU(qz)
                if (exc)
                {
                    SPARSEU(qw)
                }
            )
            IFUSED(tx, INPUTS_PER_TRANSLATION,
                // tx, ty, tz
                auto dudtx = E * (jc.KCTFR0FRFTx*X);
                auto dudty = E * (jc.KCTFR0FRFTy*X);
                auto dudtz = E * (jc.KCTFR0FRFTz*X);
                SPARSEU(tx)
                SPARSEU(ty)
                SPARSEU(tz)
            )
            IFUSED(x, INPUTS_PER_3D_POINT,
                // x, y, z
                auto dudx = E * jc.KCTFR0FRFTXx;
                auto dudy = E * jc.KCTFR0FRFTXy;
                auto dudz = E * jc.KCTFR0FRFTXz;
                SPARSEU(x)
                SPARSEU(y)
                SPARSEU(z)
            )
        }
    }
    });
    for (int i = lastProjection; i < nOut;)
    {
        auto& list = sparseDependency[i];
        IFUSED(tx, 3,
            auto fixture = positionConstrainedCameras[(i - lastProjection) / OUTPUTS_PER_POSITION_CONSTRAINT];
            auto covariation = scene->initializationData[fixture].positioningAccuracy * scalerPosition;
            for (int ii = 0; ii < 3; ++ii)
                for (int jj = 0; jj < 3; ++jj)
                    values[(&sparseDependency[i + ii].tx)[jj]] = covariation.a(ii, jj);
        )
        i += OUTPUTS_PER_POSITION_CONSTRAINT;
    }
    std::cout << std::endl;
    return SparseMatrix(getOutputNum(), getInputNum(), values, sparseCol, sparseRowptr);
}

void corecvs::ReconstructionFunctor::QuaternionDiff(double qx, double qy, double qz, double qw, corecvs::ReconstructionFunctor::QuaternionParametrization p, bool inverse, corecvs::Matrix44 &Rqx, corecvs::Matrix44 &Rqy, corecvs::Matrix44 &Rqz, corecvs::Matrix44 &Rqw)
{
    switch(p)
    {
        case QuaternionParametrization::FULL:
            Rqx = corecvs::Matrix44(   0.0, 2.0*qy, 2.0*qz, 0.0,
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
            break;
        case QuaternionParametrization::FULL_NORMALIZED:
            {
                double qx2 = qx * qx, qy2 = qy * qy, qz2 = qz * qz, qw2 = qw * qw;
                double qxqy= qx * qy, qxqz = qx * qz, qyqz = qy * qz, qxqw = qx * qw, qyqw = qy * qw, qzqw = qz * qw;
                double N = qx2 + qy2 + qz2 + qw2;
                double N2= N * N;
                Rqx = corecvs::Matrix44(
                                       4.0*qx*(qy2 + qz2), (4.0*qx*(qzqw - qxqy) + 2.0*qy*N), (-4.0*qx*(qyqw + qxqz) + 2.0*qz*N), 0,
                      (-4.0*qx*(qzqw +  qxqy) + 2.0*qy*N),               -4.0*qx*(qw2 + qy2), (-2.0*qw*N + 4.0*qx*(qxqw - qyqz)), 0,
                        (4.0*qx*(qyqw - qxqz) + 2.0*qz*N), (2.0*qw*N - 4.0*qx*(qxqw + qyqz)),                -4.0*qx*(qw2 + qz2), 0,
                                                        0,                                 0,                                  0, 0) / N2;
                Rqy = corecvs::Matrix44(
                                 -4.0*qy*(qw2 + qx2), (2.0*qx*N + 4.0*qy*(qzqw - qxqy)), (2.0*qw*N - 4.0*qy*(qyqw + qxqz)), 0,
                   (2.0*qx*N - 4.0*qy*(qzqw + qxqy)),                4.0*qy*(qx2 + qz2), (4.0*qy*(qxqw - qyqz) + 2.0*qz*N), 0,
                  (-2.0*qw*N + 4.0*qy*(qyqw - qxqz)),(-4.0*qy*(qxqw + qyqz) + 2.0*qz*N),               -4.0*qy*(qw2 + qz2), 0,
                                                   0,                                 0,                                 0, 0) / N2;
                Rqz = corecvs::Matrix44(
                                        -4.0*qz*(qw2 + qx2), (-2.0*qw*N + 4.0*qz*(qzqw - qxqy)), (2.0*qx*N - 4.0*qz*(qyqw + qxqz)), 0,
                          (2.0*qw*N - 4.0*qz*(qzqw + qxqy)),                -4.0*qz*(qw2 + qy2), (2.0*qy*N + 4.0*qz*(qxqw - qyqz)), 0,
                          (2.0*qx*N + 4.0*qz*(qyqw - qxqz)),  (2.0*qy*N - 4.0*qz*(qxqw + qyqz)),                4.0*qz*(qx2 + qy2), 0,
                                                          0,                                  0,                                 0, 0) / N2;
                Rqw = corecvs::Matrix44(
                                          4.0*qw*(qy2 + qz2),  (4.0*qw*(qzqw - qxqy) - 2.0*qz*N), (-4.0*qw*(qyqw + qxqz) + 2.0*qy*N), 0,
                          (-4.0*qw*(qzqw + qxqy) + 2.0*qz*N),                 4.0*qw*(qx2 + qz2),  (4.0*qw*(qxqw - qyqz) - 2.0*qx*N), 0,
                           (4.0*qw*(qyqw - qxqz) - 2.0*qy*N), (-4.0*qw*(qxqw + qyqz) + 2.0*qx*N),                 4.0*qw*(qx2 + qy2), 0,
                                                           0,                                  0,                                  0, 0) / N2;

            }
            break;
        case QuaternionParametrization::NON_EXCESSIVE:
            {
                double qx2 = qx * qx, qy2 = qy * qy, qz2 = qz * qz, qxqyqz = qx * qy * qz;
                double qxqy= qx * qy, qxqz = qx * qz, qyqz = qy * qz,
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
            break;
    }
    if (inverse)
    {
        Rqx.transpose();
        Rqy.transpose();
        Rqz.transpose();
        Rqw.transpose();
    }
}


void corecvs::ParallelErrorComputator::operator() (const corecvs::BlockedRange<int> &r) const
{
    auto& revDependency = functor->revDependency;
    auto& cameraCache = functor->cameraCache;
    auto& cacheRef = functor->cacheRef;
    auto& jcp = functor->cjp;
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
#define EC_(E, EE, EEE) \
    case ReconstructionFunctorOptimizationErrorType::ReconstructionFunctorOptimizationErrorType::E: \
        for (int ii = r.begin(); ii < r.end(); ++ii) \
        { \
            int i = idxs[ii * N]; \
            auto& o = *revDependency[i]; \
            auto&jc = jcp[cacheRef[i]]; \
            auto& p = o.featurePoint->reprojectedPosition; \
            auto pt = corecvs::Vector4dd(p[0], p[1], p[2], 1.0); \
            auto obs= o.observation; \
            EE \
            for (int j = 0; j < N; ++j) \
                out[ii * N + j] = EEE; \
        } \
        break;

    switch(functor->error)
    {
//      EC(REPROJECTION,  reprojectionError, e[j])
        EC_(REPROJECTION,
                auto proj = jc.KCTFR0FRFT*pt;
                proj /= proj[2];,
                proj[j]-obs[j])
        EC(ANGULAR,       angleError,        e)
        EC(CROSS_PRODUCT, crossProductError, e[j])
//        EC(RAY_DIFF,      rayDiffError,      e[j])
        EC_(RAY_DIFF,
                auto proj = jc.CTFR0FRFT*pt;
                corecvs::Vector3dd v(proj[0], proj[1], proj[2]);
                v.normalise();
                auto bproj = jc.Ki * corecvs::Vector4dd(obs[0], obs[1], 1.0, 0.0);
                corecvs::Vector3dd u(bproj[0], bproj[1], bproj[2]);
                u.normalise();,
                v[j] - u[j])


        default:
            CORE_ASSERT_TRUE_S(false);
            break;
    }
#undef EC
}
