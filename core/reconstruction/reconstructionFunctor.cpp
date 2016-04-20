#include "reconstructionFunctor.h"

#include <set>

corecvs::ReconstructionFunctor::ReconstructionFunctor(corecvs::ReconstructionFixtureScene *scene, const corecvs::PhotostationPlacerOptimizationErrorType &error, const corecvs::PhotostationPlacerOptimizationType &optimization, const double pointErrorEstimate) : corecvs::SparseFunctionArgs(), scene(scene), error(error), optimization(optimization), scalerPoints(pointErrorEstimate)
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

int corecvs::ReconstructionFunctor::getInputNum()
{
    return
        orientableFixtures.size()     * INPUTS_PER_ORIENTATION
      + translateableFixtures.size()  * INPUTS_PER_TRANSLATION
      + focalTunableCameras.size()    * INPUTS_PER_FOCAL
      + principalTunableCameras.size()* INPUTS_PER_PRINCIPAL
      + scene->trackedFeatures.size() * INPUTS_PER_3D_POINT;
}

int corecvs::ReconstructionFunctor::getOutputNum()
{
    return lastProjection + positionConstrainedCameras.size() * OUTPUTS_PER_POSITION_CONSTRAINT;
}

#define IFNOT(cond, expr) \
    if (!(optimization & PhotostationPlacerOptimizationType::cond)) \
    { \
        expr; \
    }
#define IF(cond, expr) \
    if (!!(optimization & PhotostationPlacerOptimizationType::cond)) \
    { \
        expr; \
    }
#define GETPARAM(ref) \
    ref = in[argin++];
#define IF_GETPARAM(cond, ref) \
    if (!!(optimization & PhotostationPlacerOptimizationType::cond)) ref = in[argin++];
#define IFNOT_GETPARAM(cond, ref) \
    if ( !(optimization & PhotostationPlacerOptimizationType::cond)) ref = in[argin++];
#define SETPARAM(ref) \
    out[argout++] = ref;
#define IF_SETPARAM(cond, ref) \
    if (!!(optimization & PhotostationPlacerOptimizationType::cond)) out[argout++] = ref;
#define IFNOT_SETPARAM(cond, ref) \
    if ( !(optimization & PhotostationPlacerOptimizationType::cond)) out[argout++] = ref;

void corecvs::ReconstructionFunctor::computePointCounts()
{
    counter.clear();
    for (auto& pt: scene->trackedFeatures)
        for (auto& obs: pt->observations__)
        {
            counter[obs.first.u]++;
            counter[obs.first.u]++;
        }

    for (auto& pt: scene->staticPoints)
        for (auto& obs: pt->observations__)
        {
            counter[obs.first.u]++;
            counter[obs.first.u]++;
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
        lastProjection += pt->observations__.size();
    for (auto& pt: scene->staticPoints)
        lastProjection += pt->observations__.size();

    lastProjection *= getErrorComponentsPerPoint();

    IF (TUNE_GPS,
        for (auto& cf: scene->placedFixtures)
            if (scene->initializationData[cf].enforcePosition)
                positionConstrainedCameras.push_back(cf);)

    scalerPosition = scalerPoints = 1.0;
}

int corecvs::ReconstructionFunctor::getErrorComponentsPerPoint()
{
    switch(error)
    {
        case PhotostationPlacerOptimizationErrorType::ANGULAR:
            return 1;
        case PhotostationPlacerOptimizationErrorType::REPROJECTION:
            return 2;
        case PhotostationPlacerOptimizationErrorType::CROSS_PRODUCT:
            return 3;
        case PhotostationPlacerOptimizationErrorType::RAY_DIFF:
            return 3;
    }
    CORE_ASSERT_TRUE_S(false);
}

void corecvs::ReconstructionFunctor::computeDependency()
{
    revDependency.clear();
    sparsity.clear();

    auto errSize = getErrorComponentsPerPoint();
    int id = 0, argin = 0;

    revDependency.resize(lastProjection);
    sparsity.resize(getInputNum());

#define ALL_FROM(V) \
    for (auto& t: V) \
        for (auto& o: t->observations__) \
            for (int k = 0; k < errSize; ++k) \
                revDependency[id++] = &o.second;
    ALL_FROM(scene->trackedFeatures)
    ALL_FROM(scene->staticPoints)

    CORE_ASSERT_TRUE_S(id == lastProjection);

#define DEPS(V, N, CPROJ, CPOS) \
    for (auto& a: V) \
        for (int i = 0; i < N; ++i) \
        { \
            for (int j = 0; j < lastProjection; ++j) \
            { \
                auto p = revDependency[j]; \
                if (CPROJ) \
                    sparsity[argin].push_back(j); \
            } \
            for (size_t j = 0; j < positionConstrainedCameras.size(); ++j) \
            { \
                auto p = positionConstrainedCameras[j]; \
                if (CPOS) \
                    for (int k = 0; k < OUTPUTS_PER_POSITION_CONSTRAINT; ++k) \
                        sparsity[argin].push_back(lastProjection + OUTPUTS_PER_POSITION_CONSTRAINT * j + k); \
                (void)p; \
            } \
            ++argin; \
        }
    DEPS(orientableFixtures,      INPUTS_PER_ORIENTATION, a == p->cameraFixture, false)
    DEPS(translateableFixtures,   INPUTS_PER_TRANSLATION, a == p->cameraFixture, a == p);
    DEPS(focalTunableCameras,     INPUTS_PER_FOCAL,       a == p->camera, false)
    DEPS(principalTunableCameras, INPUTS_PER_PRINCIPAL,   a == p->camera, false)
    CORE_ASSERT_TRUE_S(argin == getInputNum() - (!(optimization & PhotostationPlacerOptimizationType::POINTS) ? 0 : INPUTS_PER_3D_POINT * (int)scene->trackedFeatures.size()));
    IF(POINTS,
    DEPS(scene->trackedFeatures,  INPUTS_PER_3D_POINT,    a == p->featurePoint, false))
    CORE_ASSERT_TRUE_S(argin == getInputNum());
}

void corecvs::ReconstructionFunctor::readParams(const double* params)
{
    int argin = 0;
#define FILL(V, N, C, A) \
    for (auto& a: V) \
    { \
        for (int i = 0; i < N; ++i) \
        { \
            double v = params[argin++]; \
            C; \
        } \
        A; \
    }

    FILL(orientableFixtures,      INPUTS_PER_ORIENTATION, a->location.rotor[i] = v, a->location.rotor.normalise())
    FILL(translateableFixtures,   INPUTS_PER_TRANSLATION, a->location.shift[i] = v, )
    FILL(focalTunableCameras,     INPUTS_PER_FOCAL,       a->intrinsics.focal = corecvs::Vector2dd(v, v), )
    FILL(principalTunableCameras, INPUTS_PER_PRINCIPAL,   a->intrinsics.principal[i] = v, )
    FILL(scene->trackedFeatures,  INPUTS_PER_3D_POINT,    a->reprojectedPosition[i] = v, )
}

void corecvs::ReconstructionFunctor::writeParams(double* params)
{
    int argin = 0;
#define WRITE(V, N, C) \
    for (auto& a: V) \
        for (int i = 0; i < N; ++i) \
            params[argin++] = C;

    WRITE(orientableFixtures,      INPUTS_PER_ORIENTATION, a->location.rotor[i])
    WRITE(translateableFixtures,   INPUTS_PER_TRANSLATION, a->location.shift[i])
    WRITE(focalTunableCameras,     INPUTS_PER_FOCAL,       a->intrinsics.focal[0])
    WRITE(principalTunableCameras, INPUTS_PER_PRINCIPAL, a->intrinsics.principal[i])
    WRITE(scene->trackedFeatures,  INPUTS_PER_3D_POINT,  a->reprojectedPosition[i])
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
    corecvs::parallelable_for(0, currLastProjection / getErrorComponentsPerPoint(), 16, computator, true);

    int idx = currLastProjection;
    for (size_t i = 0; i < positions.size(); i += 3)
    {
        auto ps = positionConstrainedCameras[i / 3];

        auto diff = ps->location.shift - scene->initializationData[ps].initData.shift;
        auto foo = scene->initializationData[ps].positioningAccuracy * diff * scalerPosition;
        for (int j = 0; j < 3; ++j)
            out[idx++] = foo[i];
    }
}

void corecvs::ParallelErrorComputator::operator() (const corecvs::BlockedRange<int> &r) const
{
    auto& revDependency = functor->revDependency;
    int N = functor->getErrorComponentsPerPoint();
    double *out = output;
#define EC(E, EE, EEE) \
    case PhotostationPlacerOptimizationErrorType::E: \
        for (int ii = r.begin(); ii < r.end(); ++ii) \
        { \
            int i = idxs[ii * N]; \
            auto& o = *revDependency[i]; \
            auto  e = o.cameraFixture->EE(o.featurePoint->reprojectedPosition, o.observation, o.camera); \
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
}
