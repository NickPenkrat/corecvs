#include "sceneAligner.h"

bool corecvs::SceneAligner::TryAlign(ReconstructionFixtureScene* scene
    , corecvs::Affine3DQ &transformation
    , double &scale)
{
    if (scene->is3DAligned)
        return false;

    int gps = 0, staticFixed = 0;
    for (auto cf: scene->placedFixtures)
        switch (scene->initializationData[cf].initializationType)
        {
        case PhotostationInitializationType::GPS:
            gps++;
            break;
        case PhotostationInitializationType::STATIC:
        case PhotostationInitializationType::FIXED:
            staticFixed++;
            break;
        default:
            break;
        }
    CORE_ASSERT_TRUE_S(gps <  3 || staticFixed == 0);
    CORE_ASSERT_TRUE_S(gps <= 3 && staticFixed <= 1);
    auto lastType = scene->initializationData[*scene->placedFixtures.rbegin()].initializationType;
    if (lastType == PhotostationInitializationType::NONE)
        return false;
    CORE_ASSERT_TRUE_S(staticFixed == 0 || lastType == PhotostationInitializationType::STATIC || lastType == PhotostationInitializationType::FIXED);
    CORE_ASSERT_TRUE_S(staticFixed == 1 || lastType == PhotostationInitializationType::GPS);

    if (staticFixed)
        return TryAlignStatic(scene, transformation, scale);
    return TryAlignGPS(scene, transformation, scale);
}

corecvs::Quaternion corecvs::SceneAligner::EstimateOrientationTransformation(const corecvs::Vector3dd &e1
    , const corecvs::Vector3dd &e2
    , const corecvs::Vector3dd &o1
    , const corecvs::Vector3dd &o2)
{
    corecvs::Vector3dd e3 = e1 ^ e2;
    corecvs::Vector3dd o3 = o1 ^ o2;

    corecvs::Matrix A(9, 9);
    corecvs::Vector B(9);
    corecvs::Matrix33 RC;

    for (int i = 0; i < 3; ++i)
    {
        A.a(0, i) = A.a(1, i + 3) = A.a(2, i + 6) = o1[i];
        A.a(3, i) = A.a(4, i + 3) = A.a(5, i + 6) = o2[i];
        A.a(6, i) = A.a(7, i + 3) = A.a(8, i + 6) = o3[i];
        B[i] = e1[i];
        B[i + 3] = e2[i];
        B[i + 6] = e3[i];
    }
    corecvs::Vector Rv;
    corecvs::Matrix::LinSolve(A, B, Rv);
    int id = 0;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            RC.a(i, j) = Rv[id++];
        }
    }

    corecvs::Vector3dd v;
    corecvs::Matrix33 Vt;
    corecvs::Matrix::svd(&RC, &v, &Vt);

    corecvs::Matrix33 R = RC * Vt.transposed();
    return corecvs::Quaternion::FromMatrix(R);
}

bool corecvs::SceneAligner::TryAlignStatic(ReconstructionFixtureScene* scene
    , corecvs::Affine3DQ &transformation
    , double &scale)
{
    scale = 1.0;
    transformation = corecvs::Affine3DQ();
    scene->is3DAligned = true;
    return false;
}

bool corecvs::SceneAligner::TryAlignGPS(ReconstructionFixtureScene* scene
    , corecvs::Affine3DQ &transformation
    , double &scale)
{
    scene->printPosStats();
#if 1
    scale = 1.0;
    transformation = corecvs::Affine3DQ();

    auto expected = scene->initializationData[*scene->placedFixtures.rbegin()].initData;
    auto observed = (**scene->placedFixtures.rbegin()).location;

    std::vector<CameraFixture*> gpsFixtures;
    for (auto& cf: scene->placedFixtures)
        if (scene->initializationData[cf].initializationType == PhotostationInitializationType::GPS)
            gpsFixtures.push_back(cf);
    CORE_ASSERT_TRUE_S(gpsFixtures.size() <= 3 && gpsFixtures.size());

    size_t N = gpsFixtures.size();
    switch (N)
    {
        case 1:
            /*
             * We need to shift and the whole world in such manner
             * that this multicamera has correct coordinates (and any orientation is equally valid)
             */
            scale = 1.0;
            transformation.rotor = corecvs::Quaternion(0, 0, 0, 1);
            transformation.shift = expected.shift - observed.shift;
            break;
        case 2:
            {
                /*
             * We need to transform segment connecting multicamera centers and do not care if orientation is valid.
             * Here we may need to scale the whole world by non-unit factor
             */
            auto expected0 = scene->initializationData[gpsFixtures[0]].initData.shift;
            auto expected1 = scene->initializationData[gpsFixtures[1]].initData.shift;
            auto observed0 = gpsFixtures[0]->location.shift;
            auto observed1 = gpsFixtures[1]->location.shift;

            auto diffE = expected1 - expected0,
                 diffO = observed1 - observed0;

            scale = !diffE / !diffO;
            corecvs::Vector3dd vectors[] = {corecvs::Vector3dd(1, 0, 0), corecvs::Vector3dd(0, 1, 0), corecvs::Vector3dd(0, 0, 1)};
            int bestE = 0, bestO = 0;
            auto dotE = std::abs(vectors[0] & diffE), dotO = std::abs(vectors[0] & diffO);
            for (int i = 1; i < 3; ++i)
            {
                auto dote = std::abs(vectors[i] & diffE),
                     doto = std::abs(vectors[i] & diffO);
                if (dote < dotE)
                {
                    bestE = i;
                    dotE = dote;
                }
                if (doto < dotO)
                {
                    bestO = i;
                    dotO = doto;
                }
            }
            // Now we create orthogonal pairs in order to reuse 3-gps init
            auto e0 = diffE / !diffE, o0 = diffO / !diffO;
            auto e1 = (vectors[bestE] - e0 * (vectors[bestE] & e0)).normalised(),
                 o1 = (vectors[bestO] - o0 * (vectors[bestO] & o0)).normalised();
            transformation.rotor = EstimateOrientationTransformation(e0, e1, o0, o1);
            transformation.shift = ShiftFromRotation(transformation.rotor, scale, observed1, expected1);
    }
            break;
        case 3:
        {
            /*
             * We need to transform triangle connecting multicamera centers (and it gives us complete orientation)
             */
            auto expected0 = scene->initializationData[gpsFixtures[0]].initData.shift,
                 expected1 = scene->initializationData[gpsFixtures[1]].initData.shift,
                 expected2 = scene->initializationData[gpsFixtures[2]].initData.shift,
                 observed0 = gpsFixtures[0]->location.shift,
                 observed1 = gpsFixtures[1]->location.shift,
                 observed2 = gpsFixtures[2]->location.shift;

            auto expAB = (expected0 - expected2).normalised(),
                 expAC = (expected1 - expected2).normalised();
            auto obsAB = (observed0 - observed2).normalised(),
                 obsAC = (observed1 - observed2).normalised();

            transformation.rotor = EstimateOrientationTransformation(expAB, expAC, obsAB, obsAC);
            transformation.shift = ShiftFromRotation(transformation.rotor, scale, observed2, expected2);
        }
            break;
        default:
            CORE_ASSERT_TRUE_S(false);
    }

    scene->is3DAligned = N >= 3;
    ApplyTransformation(scene, transformation, scale);
    return scale != 1.0 || (!transformation.shift) > 0.0 || (!(transformation.rotor - transformation.rotor.conjugated())) > 0.0;
#endif
}

corecvs::Vector3dd corecvs::SceneAligner::ShiftFromRotation(const corecvs::Quaternion &Q, const double scale, const corecvs::Vector3dd &A, const corecvs::Vector3dd &B)
{
    return B / scale - (Q * A);
}

void corecvs::SceneAligner::ApplyTransformation(ReconstructionFixtureScene* scene, const corecvs::Affine3DQ &transform, const double scale)
{
    scene->printPosStats();
    scene->transform(transform, scale);
    scene->printPosStats();
    //TODO: Check how all stuff should change with scale ~= 0.0
    for (auto& cf: scene->placedFixtures)
    {
        if (scene->initializationData[cf].initializationType == PhotostationInitializationType::GPS)
            cf->location.shift = scene->initializationData[cf].initData.shift;
        if (scene->initializationData[cf].initializationType == PhotostationInitializationType::FIXED)
            cf->location = scene->initializationData[cf].initData;
    }
    scene->printPosStats();
}
