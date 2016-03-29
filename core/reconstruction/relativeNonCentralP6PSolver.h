#ifndef RELATIVENONCENTRALP6PSOLVER
#define RELATIVENONCENTRALP6PSOLVER

#include <vector>
#include <algorithm>

#include "vector3d.h"
#include "affine.h"
#include "matrix.h"

namespace corecvs
{

struct RelativeNonCentralP6PSolver
{
    static std::vector<corecvs::Affine3DQ> SolveRelativeNonCentralP6P(const std::vector<std::pair<corecvs::Vector3dd, corecvs::Vector3dd>> &LRays, const std::vector<std::pair<corecvs::Vector3dd, corecvs::Vector3dd>> &RRays);
private:
    static void SetupEliminationTemplate(corecvs::Matrix &T, const std::vector<std::pair<corecvs::Vector3dd, corecvs::Vector3dd>> &LRays, const std::vector<std::pair<corecvs::Vector3dd, corecvs::Vector3dd>> &RRays);
    static void MultiplyTemplate(corecvs::Matrix &T, corecvs::Matrix &T1);
    static void ExtractLastBasisElement(corecvs::Matrix &T1);
    static void SetupActionMatrix(corecvs::Matrix &T1, corecvs::Matrix &M);
    const static int ID[84], V1[84], V2[84], V3[84], ROW_REORDER[60];
};

}

#endif
