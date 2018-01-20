#ifndef CATADIOPTRICPROJECTION_H
#define CATADIOPTRICPROJECTION_H

#include "core/polynomial/polynomial.h"
#include "core/polynomial/polynomialSolver.h"

#include "core/math/vector/vector2d.h"
#include "core/math/vector/vector3d.h"
#include "core/function/function.h"

#include "core/cameracalibration/projection/projectionModels.h"
#include "core/xml/generated/projectionBaseParameters.h"
#include "core/xml/generated/catadioptricBaseParameters.h"


namespace corecvs{

template<class ElementType>
class GenericCatadioptricProjection : public CameraProjection {



};

/**
 *  Catadioptric Projection
 *
 *  Point in the image if formed by the ray which parameters are computed as follows
 *
 *  First direction from the principle point is estimated. It is then scaled by "focal distance".
 *  Distance to the principal point would affect the z component of the ray directon
 *
 *  \f[ \overrightarrow {q} = (\overrightarrow{p} - \overrightarrow{p}_{principal})/f \f]
 *  \f[ r = \vert \overrightarrow {q} \vert = q_x^2 + q_y^2 \f]
 *
 *  After this a polinom is computed. First coeficients is 1.0 second one is 0.0 - to keep smoothness at zero
 *
 *  \f[ z = 1 + 0 \cdot r + n_0 r^2 + n_1 r^3 + \cdots + n_i r^{i+2} \f]
 *
 *  Then the output ray is created
 *
 *  \f[ \overrightarrow d = \overrightarrow {(q_x, q_y, z)} \f]
 *  \f[ {\overrightarrow d_n} = {{\overrightarrow {d}} \over {\vert \overrightarrow{d} \vert }} \f]
 *
 *
 *  Catadioptric Projection is quite universal.
 *  For example ortographic projection is particular case of Catadioptric.
 *
 *
 *  \f[ d_n = (q_x, q_y, \sqrt {1 - q_x^2 - q_y ^ 2} ) \f]
 *
 *
 *  \f[ r = ((p_x - p_x^0)/f, (p_y - p_y^0)/f, 1.0 ) = (q_x, q_y, 1.0)\f]
 *
 *
 *  Thus we had described reverse projection, now we have to consider forward transformation
 *  generaly we need to intersect the ray with polynome in 2d
 *
 *   \f[ \overrightarrow d_n = \overrightarrow {(x_r,y_r,z_r)} \f]
 *
 *  Ray
 *    \f[ z = ({{z_r} \over {\sqrt {(x_r^2, y_r^2)}}}) r \f]
 *  Polynom
 *    \f[ z = 1 + 0 \cdot r + n_0 r^2 + n_1 r^3 + \cdots + n_i r^{i+2} \f]
 *
 *  Subtracting and solving
 *
 *    \f[ 0 = 1 - (z_r / \sqrt(x_r^2 + y_r^2)) *r + n_0 r^2 + n_1 r^3 + \cdots + n_i r^{i+2} \f]
 *
 *  We expect an abs smallest real root to be the r value we need
 *
 **/


class CatadioptricProjection : public CatadioptricBaseParameters, public CameraProjection
{
public:

    CatadioptricProjection() :
        CameraProjection(ProjectionType::CATADIOPTRIC)
    {}

    CatadioptricProjection(const Vector2dd &principal, double focal, const Vector2dd &size) :
        CatadioptricBaseParameters(principal.x(), principal.y(), focal, {} ,size.x(), size.y(), size.x(), size.y()),
        CameraProjection(ProjectionType::CATADIOPTRIC)
    {

    }


    virtual Vector2dd project(const Vector3dd &p) const override
    {
#if 0
        Vector3d n =
            point.squaredNorm() == 0.0 ? Vector3d(0, 0, 1) : point.normalized();
        double rho_tilde = n.template block<2, 1>(0, 0).norm();
        int polyN_real = polyN;
        for (polyN_real = polyN - 1; polyN_real >= 0; --polyN_real)
          if (poly[polyN_real != 0])
            break;
        polyN_real += 1;
        VectorXd poly_g(polyN_real + 2);
        /// poly w.r.t. gamma
        poly_g[0] = 1;
        poly_g[1] = -n[2];
        for (int i = 2; i < 2 + polyN_real; ++i) {
          poly_g[i] = poly[i - 2] * std::pow(rho_tilde, i);
        }
        poly_g /= poly_g[polyN_real + 1];

        Eigen::MatrixXd companion(polyN_real + 1, polyN_real + 1);
        companion.setZero();
        for (int i = 0; i < polyN_real + 1; ++i) {
          companion(i, polyN_real) = -poly_g[i];
          if (i == 0)
            continue;
          companion(i, i - 1) = 1.0;
        }
        double bestRoot = std::numeric_limits<double>::max();
        VectorXcd roots = companion.eigenvalues();
        for (int i = 0; i < roots.rows(); ++i) {
          if (roots[i].real() < 0.0)
            continue;
          if (std::abs(roots[i].imag()) > 0.0) // 1e-2 * roots[i].real())
            continue;
          bestRoot = std::min(bestRoot, roots[i].real());
        }

        if (!std::isfinite(bestRoot)) {
          bestRoot = 1.0;
        }
        return n.template block<2, 1>(0, 0) * bestRoot * intrinsics[0] +
               Vector2d(intrinsics[1], intrinsics[2]);
#else
        Vector3dd norm = p.normalised();
        vector<double> coefs;
        vector<double> roots;

        coefs.resize(mN.size() + 1);
        coefs.push_back(1.0 - norm.z());
        coefs.insert(coefs.end(), mN.begin(), mN.end());

        Polynomial poly(coefs);
        PolynomialSolver solver;
        solver.solve(poly, roots);


        return Vector2dd::Zero();
#endif

    }


    virtual Vector3dd reverse(const Vector2dd &p) const override
    {
    /*
        Map<const SE3<T>> camToWorld(pose);
        Map<const Eigen::Matrix<T, 3, 1>> intrinsics(intrinsic);
        Map<const Eigen::Matrix<T, -1, 1>> poly(catadioptric, polyN);
        Map<Eigen::Matrix<T, 3, 1>> res(residual);

        typedef Eigen::Matrix<T, 3, 1> Vector3;
        typedef Eigen::Matrix<T, 2, 1> Vector2;
        T cx(intrinsic[1]), cy(intrinsic[2]), fx(intrinsic[0]);

        Vector3 world = worldPoint.cast<T>();
        Vector3 camera = (camToWorld.inverse() * world).normalized();

        Vector2 proj = projection.cast<T>();
        proj[0] -= cx;
        proj[1] -= cy;
        proj /= fx;
        T rho = proj.norm();
        T rho2 = proj.squaredNorm();

        Eigen::Matrix<T, -1, 1> polyDeg = poly;
        polyDeg[0] = rho2;
        for (int i = 1; i < polyN; i += 2) {
          polyDeg[i] = polyDeg[i - 1] * rho;
          if (i + 1 < polyN)
            polyDeg[i + 1] = rho2 * polyDeg[i - 1];
        }
        T f(1.0);
        for (int i = 0; i < polyN; ++i)
          f += polyDeg[i] * poly[i];

        Vector3 cameraRef = Vector3(proj[0], proj[1], f).normalized();

        res = cameraRef - camera;
        return true;
    */
        Vector2dd preDeform = (p - principal()) / focal();

        double d2 = preDeform.sumAllElementsSq();
        double d  = std::sqrt(d2);

        double f = 1.0;
        double x;

        /* We actually should add starting from higher powers (smaller value) */
        x = d2;
        for (size_t i = 0; i < mN.size(); i +=2)
        {
            f += x * mN[i];
            x *= d2;
        }

        x = d * d2;
        for (size_t i = 1; i < mN.size(); i +=2)
        {
            f += x * mN[i];
            x *= d2;
        }

        return Vector3dd(preDeform.x(), preDeform.y(), f).normalised();
    }

    virtual bool isVisible(const Vector3dd &p) const override
    {
        CORE_UNUSED(p);
        return true;
    }

    /**
     * Returns target image size
     **/
    virtual Vector2dd size() const override
    {
        return  Vector2dd(sizeX(), sizeY());
    }

    virtual Vector2dd distortedSize() const override
    {
        return  Vector2dd(distortedSizeX(), distortedSizeY());
    }

    virtual Vector2dd principal() const override
    {
        return  Vector2dd(principalX(), principalY());
    }

    /* Misc */
    virtual CatadioptricProjection *clone() const override
    {
        CatadioptricProjection *p = new CatadioptricProjection();
        *p = *this;
        return p;
    }

    virtual DynamicObjectWrapper getDynamicWrapper() override
    {
        return DynamicObjectWrapper(&reflection, static_cast<CatadioptricBaseParameters *>(this));
    }




    virtual ~CatadioptricProjection() {}

};

}

#endif // CATADIOPTRICPROJECTION_H
