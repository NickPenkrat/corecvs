#include "relativeNonCentralP6PSolver.h"

#include "cblasLapackeWrapper.h"

int highMonomials[26] = { 56, 24, 26, 27, 28, 30, 31, 33, 35, 36, 37, 38, 39, 40, 42, 43, 44, 45, 46, 48, 49, 50, 51, 53, 54, 55};

// Here we build action matrix w.r.t. multiplying by v1 for G.B. elements
void corecvs::RelativeNonCentralP6PSolver::SetupActionMatrix(corecvs::Matrix &T1, corecvs::Matrix &M)
{
    CORE_ASSERT_TRUE_S(M.h == 64 && M.w == 64);
    // We store last basis element in 56'th row since T1 originally is 60x120, so it is safe
    CORE_ASSERT_TRUE_S(T1.h== 56 && T1.w==120);
    for (int i = 0; i < 26; ++i)
    {
        for (int j = 56; j < 120; ++j)
        {
            M.a(i, j - 56) = (i > 0 ? -1.0 : 1.0) * T1.a(highMonomials[i], j);
        }
    }
    M.a(26, 2) = 1.0;
    M.a(27, 3) = 1.0;
    M.a(28, 5) = 1.0;
    M.a(29, 8) = 1.0;
    M.a(30, 9) = 1.0;
    M.a(31, 10) = 1.0;
    M.a(32, 11) = 1.0;
    M.a(33, 12) = 1.0;
    M.a(34, 14) = 1.0;
    M.a(35, 15) = 1.0;
    M.a(36, 16) = 1.0;
    M.a(37, 17) = 1.0;
    M.a(38, 19) = 1.0;
    M.a(39, 20) = 1.0;
    M.a(40, 21) = 1.0;
    M.a(41, 23) = 1.0;
    M.a(42, 24) = 1.0;
    M.a(43, 26) = 1.0;
    M.a(44, 29) = 1.0;
    M.a(45, 30) = 1.0;
    M.a(46, 31) = 1.0;
    M.a(47, 32) = 1.0;
    M.a(48, 34) = 1.0;
    M.a(49, 35) = 1.0;
    M.a(50, 36) = 1.0;
    M.a(51, 38) = 1.0;
    M.a(52, 39) = 1.0;
    M.a(53, 41) = 1.0;
    M.a(54, 44) = 1.0;
    M.a(55, 45) = 1.0;
    M.a(56, 46) = 1.0;
    M.a(57, 48) = 1.0;
    M.a(58, 49) = 1.0;
    M.a(59, 51) = 1.0;
    M.a(60, 54) = 1.0;
    M.a(61, 55) = 1.0;
    M.a(62, 57) = 1.0;
    M.a(63, 60) = 1.0;
}

