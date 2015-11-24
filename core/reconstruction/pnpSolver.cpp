#ifdef WITH_BLAS
# ifdef WITH_MKL
#  include <mkl.h>
#  include <mkl_lapacke.h>
# else
#  include <cblas.h>
#  include <lapacke.h>
# endif
#else
# error Cannot build PnP solver without BLAS/LAPACK/MKL
#endif

#include "pnpSolver.h"
#include "matrix33.h"

using namespace corecvs;

std::vector<Affine3DQ> PNPSolver::solvePNP(
            std::vector<Vector3dd> &centers,
            std::vector<Vector3dd> &directions,
            std::vector<Vector3dd> &points3d)
{
    size_t N = centers.size();
    CORE_ASSERT_TRUE_S(N == directions.size());
    CORE_ASSERT_TRUE_S(N == points3d.size());
    CORE_ASSERT_TRUE_S(N >= 3);

    return solvePNP_(centers, directions, points3d);
}

std::vector<Affine3DQ> PNPSolver::solvePNP(
            std::vector<Vector3dd> &directions,
            std::vector<Vector3dd> &points3d)
{
    std::vector<Vector3dd> centers(directions.size());
    for (auto& v: centers)
        v = Vector3dd(0.0, 0.0, 0.0);
    return solvePNP(centers, directions, points3d);
}

std::vector<Affine3DQ> PNPSolver::solvePNP_(
        std::vector<Vector3dd> &centers,
        std::vector<Vector3dd> &directions,
        std::vector<Vector3dd> &points3d)
{
    // First step is an O(n) reduction of problem to non n-dependent
    Matrix33 F(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    size_t N = centers.size();

    for (auto& v: directions)
    {
//        std::cout << v << std::endl;
        F = F + Matrix33::VectorByVector(v, v);
    }

    Matrix33 H = (N * Matrix33::Identity() - F).inv();
//    std::cout << "F: " << F << std::endl << "H: " << H << std::endl;

    Matrix I(3, 10);
    Vector J(3);
    J[0] = J[1] = J[2] = 0.0;
    Matrix Phi(3, 10);

    for (size_t i = 0; i < N; ++i)
    {
        auto f  = directions[i];
        // XXX: due to stupid "bugs" (which A.Pimenov calls
        // "design considerations") in corecvs we cannot multiply Matrix*Matrix33
        // and Matrix * Vector3dd
        // so here we cast!
        Vector3dd vv = centers[i];
        Matrix Vk(H * (Matrix33::VectorByVector(f, f) - Matrix33::Identity())); 
        auto p = points3d[i];
        Vector v(centers[i]);

        fillPhi(p, Phi);


        I += Vk * Phi;
        J += Vk * v;
    }

    Matrix M(10, 10);
    Vector C(10);
    for (int i = 0; i < 10; ++i) C[i] = 0.0;
    double gamma = 0.0;

    for (size_t i = 0; i < N; ++i)
    {
        auto f = directions[i];
        auto p = points3d[i];
        Vector v(centers[i]);

        fillPhi(p, Phi);
        // XXX: due to stupid "bugs" (which A.Pimenov calls
        // "design considerations") in corecvs we cannot multiply Matrix*Matrix33
        // and Matrix * Vector3dd
        // so here we cast!
        Matrix foo(Matrix33::VectorByVector(f, f) - Matrix33::Identity());
        Matrix Ai = foo * (Phi + I);
        Vector bi =-foo * (  v + J);

        M += (Ai.t() * Ai);
        C += (bi * Ai);
        gamma += (bi & bi);
    }

    Matrix CC(1, 10);
    for (int i = 0; i < 10; ++i) CC.a(0, i) = C[i];
    // Now we call solver
    std::vector<std::pair<double, corecvs::Vector4dd>> quaternionsA;
    if ( N > 4)
        solvePNPImpl (M, CC, gamma, quaternionsA);
    else
        solveP34PImpl(M, CC, gamma, quaternionsA);

    // Now we filter out some roots as whell as reconstruct translations from rotations
    std::vector<corecvs::Affine3DQ> transformations;
    bool isFirst = true;
    corecvs::Affine3DQ first;
    for (auto& pv: quaternionsA)
    {
        Vector4dd qAlt = pv.second;
        Vector s(10);
        fillS(qAlt, s);
        Vector tinv = I * s - J;
        Vector3dd  ti(tinv[0], tinv[1], tinv[2]);
        Quaternion q(qAlt[1], qAlt[2], qAlt[3], qAlt[0]);

        if (isFirst)
            first = corecvs::Affine3DQ(q.conjugated(), -(q.conjugated() * ti));

        size_t neg = 0;
        for (size_t i = 0; i < N; ++i)
        {
            auto f = directions[i];
            auto p = points3d[i];
            auto v = centers[i];

            auto est = q * p + ti - v;

            if ((est & f) < 0.0)
                neg++;
        }

        if (neg == 0)
        {
            transformations.emplace_back(q.conjugated(), -(q.conjugated() * ti));
        }
        else
        {
//            std::cout << "Hypo " << pv.second << " failed chirality test" << std::endl;
        }
    }
    if (!transformations.size())
        transformations.push_back(first);
    return transformations;
}

void PNPSolver::solveP34PImpl(corecvs::Matrix &M, corecvs::Matrix &C, double gamma, std::vector<std::pair<double, corecvs::Vector4dd>> &quaternions)
{
    Matrix gbAction(16, 16);
    setupP34PgbActionMatrix(M, C, gamma, gbAction);
    Vector evr(16), evi(16);
    Matrix EV(16, 16);

    LAPACKE_dgeev(LAPACK_ROW_MAJOR, 'N', 'V', gbAction.w, &gbAction.a(0, 0), gbAction.stride, &evr[0], &evi[0], 0, gbAction.w, &EV.a(0, 0), EV.stride);
    
    for (int i = 0; i < 16; ++i)
    {
        // In order to decrease sensitivity to noise we make a strange thing here:
        // We try a solution even if it is non-real
        bool isReal = std::abs(evi[i]) == 0.0;

        corecvs::Vector4dd quaternion;
        double norm = 0.0;
        for (int q = 0; q < 4; ++q)
        {
            // We check only real part of EV, and increment twice on
            // non-real values, so it is ok.
            quaternion[q] = EV.a(11 + q, i);
            norm += quaternion[q] * quaternion[q];
        }
        norm = std::sqrt(norm);
        if (quaternion[0] < 0.0)
            norm *= -1.0;
        quaternion *= 1. / norm;

        bool alreadyAdded = false;
        for (auto& v: quaternions)
        {
            if (!(v.second - quaternion) < ELIMINATE_THRESHOLD)
            {
                alreadyAdded = true;
                break;
            }
        }

        if (!alreadyAdded)
        {
            Vector s(10);
            fillS(quaternion, s);
            double vM = (s & (M * s)) + (2.0 * C * s)[0];
            double value = vM + gamma;
            quaternions.push_back(std::make_pair(value, quaternion));
        }

        if (!isReal)
            ++i;
    }
    std::sort(quaternions.begin(), quaternions.end(), [](const std::pair<double, corecvs::Vector4dd> &a, const std::pair<double, corecvs::Vector4dd> &b) { return a.first < b.first; });
}

void PNPSolver::solvePNPImpl(corecvs::Matrix &M, corecvs::Matrix &C, double gamma, std::vector<std::pair<double, corecvs::Vector4dd>> &quaternions)
{
    Matrix gbAction(8, 8);
    setupPNPgbActionMatrix(M, C, gamma, gbAction);
    Vector evr(8), evi(8);
    Matrix EV(8, 8);

    LAPACKE_dgeev(LAPACK_ROW_MAJOR, 'N', 'V', gbAction.w, &gbAction.a(0, 0), gbAction.stride, &evr[0], &evi[0], 0, gbAction.w, &EV.a(0, 0), EV.stride);

    for (int i = 0; i < 8; ++i)
    {
        bool isReal = std::abs(evi[i]) == 0.0;
        corecvs::Vector4dd quaternion;
        
        double norm = 0.0;
        for (int q = 0; q < 4; ++q)
        {
            // We check only real part of EV, and increment twice on
            // non-real values, so it is ok.
            quaternion[q] = EV.a(4 + q, i);
            norm += quaternion[q] * quaternion[q];
        }
        norm = std::sqrt(norm);
        if (quaternion[0] < 0.0)
            norm *= -1.0;
        quaternion *= 1. / norm;

        bool alreadyAdded = false;
        for (auto& v: quaternions)
        {
            if (!(v.second - quaternion) < ELIMINATE_THRESHOLD)
            {
                alreadyAdded = true;
                break;
            }
        }

        if (!alreadyAdded)
        {
            Vector s(10);
            fillS(quaternion, s);
            double vM = (s & (M * s)) + (2.0 * C * s)[0];
            double value = vM + gamma;
            quaternions.push_back(std::make_pair(value, quaternion));
        }

        if (!isReal)
            ++i;

    }
    std::sort(quaternions.begin(), quaternions.end(), [](const std::pair<double, corecvs::Vector4dd> &a, const std::pair<double, corecvs::Vector4dd> &b) { return a.first < b.first; });
}

void PNPSolver::fillPhi(const corecvs::Vector3dd &p, corecvs::Matrix &m)
{
    CORE_ASSERT_TRUE_S(m.h == 3 && m.w == 10);
    // Monomials (quaternion here is q0 + i(q1 q2 q3), not this corecvs rubbish)
    // q0^2 q1^2 q2^2 q3^2 q0q1 q0q2 q0q3 q1q2 q1q3 q2q3 \in \mathbb{R}^{10}
    double x = p[0], y = p[1], z = p[2];
    // Do not alter this lines for readability reasons
#define M(b, c, d) m.a(b, c) = d;
    M(0, 0, x); M(0, 1, x); M(0, 2, -x); M(0, 3, -x); M(0, 4, 0);       M(0, 5, 2.0 * z); M(0, 6, -2.0 * y); M(0, 7, 2.0 * y); M(0, 8, 2.0 * z); M(0, 9,     0.0);
    M(1, 0, y); M(1, 1,-y); M(1, 2,  y); M(1, 3, -y); M(1, 4,-2.0 * z); M(1, 5,     0.0); M(1, 6,  2.0 * x); M(1, 7, 2.0 * x); M(1, 8,     0.0); M(1, 9, 2.0 * z);
    M(2, 0, z); M(2, 1,-z); M(2, 2, -z); M(2, 3,  z); M(2, 4, 2.0 * y); M(2, 5,-2.0 * x); M(2, 6,      0.0); M(2, 7,     0.0); M(2, 8, 2.0 * x); M(2, 9, 2.0 * y);
#undef M
}

void PNPSolver::fillS(const corecvs::Vector4dd &q, corecvs::Vector &v)
{
    CORE_ASSERT_TRUE_S(v.size() == 10);
    double q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    v[0] = q0 * q0;
    v[1] = q1 * q1;
    v[2] = q2 * q2;
    v[3] = q3 * q3;
    v[4] = q0 * q1;
    v[5] = q0 * q2;
    v[6] = q0 * q3;
    v[7] = q1 * q2;
    v[8] = q1 * q3;
    v[9] = q2 * q3;
}

