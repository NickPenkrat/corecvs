#ifndef PNPSOLVER_H
#define PNPSOLVER_H

#include <vector>

#include "vector.h"
#include "matrix.h"
#include "affine.h"

namespace corecvs
{
    /*
     * This class solves PnP problem for central (projective)
     * and non-central (e.g. photostations, non-central cameras)
     * cameras using known 3D-points
     *
     * It requires at least 3 points but can run on arbitrary
     * number of points in O(n) complexity
     *
     * It is based on algebraic (system of multi-variable polynomial
     * equations) formulation and uses Groebner bases approach
     * to solve this problem.
     *
     * Since we do not need to construct Groebner bases every time
     * from scratch we use pregenerated code and our problem reduces
     * to
     * 1. Transform problem into rotation-estimation problem
     * 2. Make some GJ-eliminations and solve EV-problems
     * 3. Decompose rotations from real roots for variables
     * 4. Solve translations from rotations
     * 5. Enforce chirality constraints
     *
     * Note that this method can return up to 4 results in minimal
     * (3-point) case due to geometric nature of task being solved
     */
struct PNPSolver
{
    //! This method runs solver
    static std::vector<corecvs::Affine3DQ> solvePNP(
            std::vector<corecvs::Vector3dd> &centers,
            std::vector<corecvs::Vector3dd> &directions,
            std::vector<corecvs::Vector3dd> &points3d);
    //! Shortcut for central cameras; centers will be initalized with 0s
    static std::vector<corecvs::Affine3DQ> solvePNP(
            std::vector<corecvs::Vector3dd> &directions,
            std::vector<corecvs::Vector3dd> &points3d);
private:
    static const double INLIER_CHIRALITY;
    static const double ELIMINATE_THRESHOLD;
    //! Real solver
    static std::vector<corecvs::Affine3DQ> solvePNP_(
            std::vector<corecvs::Vector3dd> &centers,
            std::vector<corecvs::Vector3dd> &directions,
            std::vector<corecvs::Vector3dd> &points3d);
    //! Separate implementation for minimal case
    static void solveP34PImpl(
            corecvs::Matrix &M,
            corecvs::Matrix &C,
            double gamma,
            std::vector<std::pair<double, corecvs::Vector4dd>> &quaternions);
    //! General-purpose implementation
    static void solvePNPImpl(
            corecvs::Matrix &M,
            corecvs::Matrix &C,
            double gamma,
            std::vector<std::pair<double, corecvs::Vector4dd>> &quaternions);
    //! Setups action matrix and runs elimination templates 
    static void setupP34PgbActionMatrix(
            corecvs::Matrix &M, 
            corecvs::Matrix &C, 
            double gamma, 
            corecvs::Matrix &gbAction);
    static void setupPNPgbActionMatrix(
            corecvs::Matrix &M, 
            corecvs::Matrix &C, 
            double gamma, 
            corecvs::Matrix &gbAction);
    //! Creates matrix for expressing rotation as multiplying by quaternion
    //  quadratic monomials vector
    static void fillPhi(const corecvs::Vector3dd &p, corecvs::Matrix &m);
    //! Creates vector for quaternion quadratic monomials
    static void fillS  (const corecvs::Vector4dd &q, corecvs::Vector &v);
};
}
#endif
