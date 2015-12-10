#include "relativeNonCentralP6PSolver.h"

#ifdef WITH_BLAS
# ifdef WITH_MKL
#  include <mkl.h>
#  include <mkl_lapacke.h>
# else
#  include <cblas.h>
#  include <lapacke.h>
# endif
#else
# error Cannot build solver without BLAS/LAPACK/MKL
#endif

#if 0
int ID[]={35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119};
int V1[]={0,1,2,3,4,5,6,8,9,10,11,12,13,15,16,17,18,19,21,22,23,24,26,27,28,30,31,33,35,36,37,38,39,40,42,43,44,45,46,48,49,50,51,53,54,55,58,59,61,64,65,66,67,68,70,71,72,73,75,76,77,79,80,82,85,86,87,88,90,91,92,94,95,97,100,101,102,104,105,107,110,111,113,116};
int V2[]={1,2,3,4,5,6,7,9,10,11,12,13,14,16,17,18,19,20,22,23,24,25,27,28,29,31,32,34,36,37,38,39,40,41,43,44,45,46,47,49,50,51,52,54,55,57,59,60,62,65,66,67,68,69,71,72,73,74,76,77,78,80,81,83,86,87,88,89,91,92,93,95,96,98,101,102,103,105,106,108,111,112,114,117};
int V3[]={8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,56,42,43,44,45,46,47,48,49,50,51,52,53,54,55,57,58,59,60,61,62,63,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,90,91,92,93,94,95,96,97,98,99,104,105,106,107,108,109,113,114,115,118};
#endif

// Last element for GB basis is v3*T1(32,:)-v2*T1(34,:), then we should eliminate it.
void corecvs::RelativeNonCentralP6PSolver::ExtractLastBasisElement(corecvs::Matrix &T1)
{
    CORE_ASSERT_TRUE_S(T1.h == 56 && T1.w == 120);
    double *last = &T1.a(57, 0);
    double *rest = &T1.a(56, 0);
    memset(last, 0, sizeof(double) * 120);
    memset(rest, 0, sizeof(double) * 120);
    for (int i = 0; i < 84; ++i)
    {
        last[V3[i]] += T1.a(32, ID[i]);
        last[V2[i]] -= T1.a(34, ID[i]);
    }
    // and few monomials outside of matrix
    double c017 = -T1.a(34, 56),
           c008 =  T1.a(32, 56);
    c008 -= c017 * T1.a(34, 56);

    for (int i = 0; i < 84; ++i)
    {
        last[V3[i]] -= c017 * T1.a(34, ID[i]);
    }
    for (int i = 0; i < 57; ++i)
    {
        double t = last[i];
        for (int j = 0; j < 120; ++j)
            last[j] -= t * T1.a(i, j);
    }
    for (int i = 0; i < 120; ++i)
    {
        last[i] /= c008;
    }

    double v38 = -T1.a(33, 56);
    for (int i = 0; i < 84; ++i)
    {
        rest[V3[i]] = -T1.a(33, ID[i]);
    }

    for (int i = 0; i < 56; ++i)
    {
        double t = rest[i];
        for (int j = 0; j < 120; ++j)
        {
            rest[j] -= t * T1.a(i, j);
        }
    }

    for (int i = 0; i < 120; ++i)
    {
        rest[i] -= v38 * last[i];
    }
}

