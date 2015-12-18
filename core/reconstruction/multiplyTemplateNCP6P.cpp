#include "relativeNonCentralP6PSolver.h"

#include "cblasLapackeWrapper.h"

/*
 * Here we take polynomials in T (in GRevLex order, 3 vars x 6th degree = C(3+1-1, 3+1+6-1)=C(3, 9)=84 monomials)
 * and multiply them by v_1, v_2, v_3 and 1 and store in T1, 3 vars x 7th degree = C(3+1-1, 3+1+7-1)=C(3, 10)=120 monomials)
 *
 * We store indicies for monomial transformation (V1..V3,ID) as well as their reorder (since we already know how leading 1's positioned in T1) in ROW_REORDER
 */
const int corecvs::RelativeNonCentralP6PSolver::V1[84]={0,1,2,3,4,5,6,8,9,10,11,12,13,15,16,17,18,19,21,22,23,24,26,27,28,30,31,33,35,36,37,38,39,40,42,43,44,45,46,48,49,50,51,53,54,55,58,59,61,64,65,66,67,68,70,71,72,73,75,76,77,79,80,82,85,86,87,88,90,91,92,94,95,97,100,101,102,104,105,107,110,111,113,116};
const int corecvs::RelativeNonCentralP6PSolver::V2[84]={1,2,3,4,5,6,7,9,10,11,12,13,14,16,17,18,19,20,22,23,24,25,27,28,29,31,32,34,36,37,38,39,40,41,43,44,45,46,47,49,50,51,52,54,55,57,59,60,62,65,66,67,68,69,71,72,73,74,76,77,78,80,81,83,86,87,88,89,91,92,93,95,96,98,101,102,103,105,106,108,111,112,114,117};
const int corecvs::RelativeNonCentralP6PSolver::V3[84]={8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,56,42,43,44,45,46,47,48,49,50,51,52,53,54,55,57,58,59,60,61,62,63,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,90,91,92,93,94,95,96,97,98,99,104,105,106,107,108,109,113,114,115,118};
const int corecvs::RelativeNonCentralP6PSolver::ID[84]={35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119};
const int corecvs::RelativeNonCentralP6PSolver::ROW_REORDER[60]={0,1,2,3,4,5,6,8,9,10,11,12,13,15,16,22,23,24,25,26,27,7,28,29,30,31,32,14,33,17,34,35,36,37,38,39,40,41,42,43,44,18,19,20,21,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59};


void corecvs::RelativeNonCentralP6PSolver::MultiplyTemplate(corecvs::Matrix &T, corecvs::Matrix &T1)
{
    CORE_ASSERT_TRUE_S (T.h == 15 && T.w == 84);
    CORE_ASSERT_TRUE_S(T1.h == 60 && T1.w==120);
    int N = 15;
    
    for (int i = 0; i < N; ++i)
    {
        int reorderID = ROW_REORDER[N * 3 + i];
        int reorderV3 = ROW_REORDER[N * 2 + i];
        int reorderV2 = ROW_REORDER[N * 1 + i];
        int reorderV1 = ROW_REORDER[N * 0 + i];
        for (int j = 0; j < 84; ++j)
        {
            double coeff = T.a(i, j);
            T1.a(reorderID, ID[j]) = coeff;
            T1.a(reorderV1, V1[j]) = coeff;
            T1.a(reorderV2, V2[j]) = coeff;
            T1.a(reorderV3, V3[j]) = coeff;
        }
    }
}

