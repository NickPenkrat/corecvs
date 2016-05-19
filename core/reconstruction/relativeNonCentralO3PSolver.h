#ifndef RELATIVENONCENTRALO3PSOLVER
#define RELATIVENONCENTRALO3PSOLVER

#include <vector>
#include <algorithm>

#include "vector3d.h"
#include "affine.h"
#include "matrix.h"


namespace corecvs
{
struct RelativeNonCentralO3PSolver
{
    static std::vector<corecvs::Affine3DQ> SolveRelativeNonCentralO3P(const std::vector<std::pair<corecvs::Vector3dd, corecvs::Vector3dd>> &LRays, const std::vector<std::pair<corecvs::Vector3dd, corecvs::Vector3dd>> &RRays, const corecvs::Vector3dd &shift);
private:
    static void SetupEliminationTemplate(corecvs::Matrix &T, const std::vector<std::pair<corecvs::Vector3dd, corecvs::Vector3dd>> &LRays, const std::vector<std::pair<corecvs::Vector3dd, corecvs::Vector3dd>> &RRays, const corecvs::Vector3dd &shift);
    static void MultiplyFirstTemplate (const corecvs::Matrix &T,  corecvs::Matrix &T1);
    static void MultiplySecondTemplate(const corecvs::Matrix &T1, corecvs::Matrix &T2);
    static void SetupActionMatrix     (const corecvs::Matrix &T2, corecvs::Matrix &M);
    static void Eliminate(corecvs::Matrix &T, int lastN = -1);
};

}


#endif
