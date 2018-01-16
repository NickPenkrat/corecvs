#ifndef CATADIOPTRICPROJECTION_H
#define CATADIOPTRICPROJECTION_H

#include "vector2d.h"
#include "vector3d.h"

#include "projectionModels.h"
#include "projectionBaseParameters.h"

namespace corecvs{

template<class ElementType>
class GenericCatadioptricProjection : public CameraProjection {



};

#if 0
class CatadioptricProjection : public ProjectionBaseParameters, public CameraProjection
{
public:
    CatadioptricProjection(const Vector2dd &principal, double focal) :
        ProjectionBaseParameters(principal.x(), principal.y(), focal)
    {

    }


    virtual Vector2dd project(const Vector3dd &p) const override
    {
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
    }

    virtual Vector3dd reverse(const Vector2dd &p) const override
    {
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
    }

};
#endif

}

#endif // CATADIOPTRICPROJECTION_H
