#ifndef CATADIOPTRICPROJECTION_H
#define CATADIOPTRICPROJECTION_H

#include "core/polynomial/polynomial.h"

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


class CatadioptricProjection : public CatadioptricBaseParameters, public CameraProjection
{
public:
    CatadioptricProjection(const Vector2dd &principal, double focal, const Vector2dd &size) :
        CatadioptricBaseParameters(principal.x(), principal.y(), focal, {} ,size.x(), size.y(), size.x(), size.y()),
        CameraProjection(CATADIOPTRIC)
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
        CORE_UNUSED(p);
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
        double x = d;

        /* We actually should add starting from higher powers (smaller value) */
        for (size_t i = 0; i < mR.size(); i +=2)
        {
            f += x * mR[i];
            x *= d2;
        }

        x = d2;
        for (size_t i = 1; i < mR.size(); i +=2)
        {
            f += x * mR[i];
            x *= d2;
        }
        return Vector3dd(preDeform.x(), preDeform.y(), f).normalised();
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

};

}

#endif // CATADIOPTRICPROJECTION_H
