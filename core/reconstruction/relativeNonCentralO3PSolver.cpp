#include "relativeNonCentralO3PSolver.h"

#include "cblasLapackeWrapper.h"
#include <complex>

std::vector<corecvs::Affine3DQ> corecvs::RelativeNonCentralO3PSolver::SolveRelativeNonCentralO3P(const std::vector<std::pair<corecvs::Vector3dd, corecvs::Vector3dd>> &LRays, const std::vector<std::pair<corecvs::Vector3dd, corecvs::Vector3dd>> &RRays, const corecvs::Vector3dd &shift)
{
    corecvs::Matrix  T(200, 330),
                     M(16, 16);
    SetupEliminationTemplate(T, LRays, RRays, shift);
    Eliminate(T);
    SetupActionMatrix(T, M);

    corecvs::Matrix EV(16, 16);
    corecvs::Vector evr(16), evi(16);
    LAPACKE_dgeev(LAPACK_ROW_MAJOR, 'N', 'V', M.w, &M.a(0, 0), M.stride, &evr[0], &evi[0], 0, M.w, &EV.a(0, 0), EV.stride);

    std::vector<corecvs::Affine3DQ> res;
    for (int i = 0; i < 16; ++i)
    {
        bool isReal = std::abs(evi[i]) == 0.0;
        std::complex<double> vc[4];
        for (int j = 0; j < 4; ++j)
        {
            vc[j] = isReal ? EV.a(11 + j, i) / EV.a(15, i)
                           : std::complex<double>(EV.a(11 + j, i), EV.a(11 + j, i + 1)) / std::complex<double>(EV.a(15, i), EV.a(15, i + 1));
        }
        corecvs::Quaternion q;
        for (int j = 0; j < 4; ++j)
            q[j] = vc[j].real();
//        if (isReal)
        res.emplace_back(q.conjugated().normalised(), shift);
        if (!isReal)
            i++;
    }
    return res;
}

void corecvs::RelativeNonCentralO3PSolver::Eliminate(corecvs::Matrix &T, int lastN)
{
    std::unique_ptr<int[]> pivot(new int[T.w]);
    LAPACKE_dgetrf(LAPACK_ROW_MAJOR, T.h, T.w, &T.a(0, 0), T.stride, pivot.get());
    for (int i = 0; i < T.h; ++i)
        for (int j = 0; j < i; ++j)
            T.a(i, j) = 0.0;
    corecvs::Matrix CT(T, T.h, 0, T.w, T.h);
    LAPACKE_dtrtrs(LAPACK_ROW_MAJOR, 'U', 'N', 'N', T.h, T.w - T.h, &T.a(0, 0), T.stride, &CT.a(0, 0), CT.stride);
    for (int i = 0; i < T.h; ++i)
        for (int j = 0; j < T.h; ++j)
            T.a(i, j) = i == j ? 1.0 : 0.0;
    for (int i = 0; i < T.h; ++i)
        for (int j = T.h; j < T.w; ++j)
            T.a(i, j) = CT.a(i, j - T.h);
}
