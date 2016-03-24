#include "relativeNonCentralP6PSolver.h"

#include "cblasLapackeWrapper.h"

#include <complex>


std::vector<corecvs::Affine3DQ> corecvs::RelativeNonCentralP6PSolver::SolveRelativeNonCentralP6P(const std::vector<std::pair<corecvs::Vector3dd, corecvs::Vector3dd>> &LRays, const std::vector<std::pair<corecvs::Vector3dd, corecvs::Vector3dd>> &RRays)
{
    corecvs::Matrix T(30, 84), // 30 equ x C(3+1-1, 6+3+1-1)=C(3, 9)=84 monomials
        T1(60, 120), // T -> 15 lineary independent eq's -> x4 -> 60 equ x C(3+1-1, 7+3+1-1)=C(3, 10)=120 monomials
        M (64, 64); // R/I -> 64-dim
    int pivot[60];
    SetupEliminationTemplate(T, LRays, RRays);
    LAPACKE_dgetrf(LAPACK_ROW_MAJOR, T.h, T.w, &T.a(0, 0), T.stride, pivot);
    for (int i = 0; i < 15; ++i)
        for (int j = 0; j < i; ++j)
            T.a(i, j) = 0.0;
    corecvs::Matrix CT(T);
    LAPACKE_dtrtrs(LAPACK_ROW_MAJOR, 'U', 'N', 'N', 15, T.w, &CT.a(0, 0), CT.stride, &T.a(0, 0), T.stride);
    T.h = 15;

    MultiplyTemplate(T, T1);

    LAPACKE_dgetrf(LAPACK_ROW_MAJOR, T1.h, T1.w, &T1.a(0, 0), T1.stride, pivot);
    for (int i = 0; i < 56; ++i)
        for (int j = 0; j < i; ++j)
            T1.a(i, j) = 0.0;
    corecvs::Matrix CT1(T1);
    T1.h = 56;
    LAPACKE_dtrtrs(LAPACK_ROW_MAJOR, 'U', 'N', 'N', 56, T1.w, &CT1.a(0, 0), CT1.stride, &T1.a(0, 0), T1.stride);
   
    ExtractLastBasisElement(T1);
    SetupActionMatrix(T1, M);

    corecvs::Matrix EV(64, 64);
    corecvs::Vector evr(64), evi(64);
    LAPACKE_dgeev(LAPACK_ROW_MAJOR, 'N', 'V', M.w, &M.a(0, 0), M.stride, &evr[0], &evi[0], 0, M.w, &EV.a(0, 0), EV.stride);

    std::vector<corecvs::Affine3DQ> res;
    for (int i = 0; i < 64; ++i)
    {
        bool isReal = std::abs(evi[i]) == 0.0;
        corecvs::Vector3dd v;
        std::complex<double> vc[3];
        for (int j = 0; j < 3; ++j)
        {
            if (isReal)
            {
                vc[j] = EV.a(60 + j, i) / EV.a(63, i);
            }
            else
            {
                vc[j] = std::complex<double>(EV.a(60+j,i),EV.a(60+j,i+1)) / std::complex<double>(EV.a(63, i) / EV.a(63, i+1));
            }
        }
        for (int j = 0; j < 3; ++j)
        {
            v[j] = vc[j].real();
        }
        if (1)//:abs(evi[i]) < 1e-5 * std::abs(evr[i]))
        {
            corecvs::Vector V(6);
            corecvs::Quaternion q = corecvs::Quaternion(v[0], v[1], v[2], 1).normalised();
            corecvs::Matrix33 R = q.toMatrix().transposed();
            double r11, r12, r13, r21, r22, r23, r31, r32, r33,
                   q11, q12, q13, q21, q22, q23,
                   q1t1,q1t2,q1t3,q2t1,q2t2,q2t3;

            r11 = R.a(0, 0); r12 = R.a(0, 1); r13 = R.a(0, 2);
            r21 = R.a(1, 0); r22 = R.a(1, 1); r23 = R.a(1, 2);
            r31 = R.a(2, 0); r32 = R.a(2, 1); r33 = R.a(2, 2);
            corecvs::Matrix MM(6, 3);
            for (int j = 0; j < 6; ++j)
            {
                auto Q1 = LRays[j].first;
                auto Q1t= LRays[j].second;
                auto Q2 = RRays[j].first;
                auto Q2t= RRays[j].second;
                q11 = Q1[0]; q12 = Q1[1]; q13 = Q1[2];
                q1t1=Q1t[0];q1t2 =Q1t[1];q1t3 =Q1t[2];
                q21 = Q2[0]; q22 = Q2[1]; q23 = Q2[2];
                q2t1=Q2t[0];q2t2 =Q2t[1];q2t3 =Q2t[2];
                double cx, cy, cz, c1;
                cx = q13*(q21*r12 + q22*r22 + q23*r32) - q12*(q21*r13 + q22*r23 + q23*r33);
                cy = q11*(q21*r13 + q22*r23 + q23*r33) - q13*(q21*r11 + q22*r21 + q23*r31);
                cz = q12*(q21*r11 + q22*r21 + q23*r31) - q11*(q21*r12 + q22*r22 + q23*r32);
                c1 = q2t1*(q11*r11 + q12*r12 + q13*r13) + q2t2*(q11*r21 + q12*r22 + q13*r23) + q1t1*(q21*r11 + q22*r21 + q23*r31) + q1t2*(q21*r12 + q22*r22 + q23*r32) + q1t3*(q21*r13 + q22*r23 + q23*r33) + q2t3*(q11*r31 + q12*r32 + q13*r33);
                MM.a(j, 0) = cx;
                MM.a(j, 1) = cy;
                MM.a(j, 2) = cz;
                V[j] =-c1;
            }
            corecvs::Vector VV;
            corecvs::Matrix::LinSolve(MM.t() * MM, MM.t() * V, VV, true);
            corecvs::Vector3dd vv(VV);
            res.emplace_back(q, vv);
        }

        if (!isReal)
        {
            ++i;
        }
    }
    return res;
}
