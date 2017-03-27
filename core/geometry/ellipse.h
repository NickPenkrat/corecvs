#ifndef ELLIPSE_H
#define ELLIPSE_H

#include "vector2d.h"
#include "polynomial.h"
#include "affine.h"
#include "line.h"
#include "conic.h"

namespace corecvs {


class Ellipse;

/**
    Second Order Polinomial Curve (should we call this Quadric? )

    \f[ a_{11}x^{2}+a_{22}y^{2}+2a_{12}xy+2a_{13}x+2a_{23}y+a_{33}=0 \f]

    Internal form stores a_ij directly without coefficient 2

**/
class SecondOrderCurve  : public FixedVector<double, 6>
{
public:
    Matrix33 toMatrixForm() const;

    /** different convensions on field naming */
      double A()       { return element[0]; }
      double B()       { return element[1]; }
      double C()       { return 2 * element[2]; }
      double D()       { return 2 * element[3]; }
      double E()       { return 2 * element[4]; }
      double F()       { return element[5]; }
    /** different convensions on field naming */
          double &a11()       { return element[0]; }
    const double &a11() const { return element[0]; }
          double &a12()       { return element[1]; }
    const double &a12() const { return element[1]; }
          double &a21()       { return element[1]; }
    const double &a21() const { return element[1]; }

          double &a22()       { return element[2]; }
    const double &a22() const { return element[2]; }

          double &a13()       { return element[3]; }
    const double &a13() const { return element[3]; }
          double &a31()       { return element[3]; }
    const double &a31() const { return element[3]; }

          double &a23()       { return element[4]; }
    const double &a23() const { return element[4]; }
          double &a32()       { return element[4]; }
    const double &a32() const { return element[4]; }

          double &a33()       { return element[5]; }
    const double &a33() const { return element[5]; }


    /* Transformation*/
    void transform(const Matrix33 & B);
    SecondOrderCurve transformed(const Matrix33 &B);



    /* Invariants */
    double invDelta() const;
    double invD() const;
    double invI() const;

    /* Classifiaction */
    enum CurveClass {
        ELLIPSE,
        CIRCLE,
        IMAGINARTY_ELLIPSE,
        HYPERBOLA,
        PARABOLA,
        DEGENERATE, /**< no you.*/
    };

    static inline const char *getName(const CurveClass &value)
    {
        switch (value)
        {
         case ELLIPSE            : return "ELLIPSE"           ; break ;
         case CIRCLE             : return "CIRCLE"            ; break ;
         case IMAGINARTY_ELLIPSE : return "IMAGINARTY_ELLIPSE"; break ;
         case HYPERBOLA          : return "HYPERBOLA"         ; break ;
         case PARABOLA           : return "PARABOLA"          ; break ;
         case DEGENERATE         : return "DEGENERATE"        ; break ;
         default : return "Not in range"; break ;
        }
        return "Not in range";
    }


    CurveClass classify(double epsilon = 1e-10);

    /* Tangents and polars */
    Vector2dd tangent(Vector2dd &point) const;
    Line2d    polar  (Vector2dd &point) const;

    /* Inside */
    double value (const Vector2dd &point) const;
    double value (double x, double y) const;
    bool isInside(const Vector2dd &point) const;
    bool isInside(double x, double y) const;

    /* SecondOrderCurve */
    static SecondOrderCurve FromCircle(const Vector2dd &center, double r);
    static SecondOrderCurve FromCircle(const Circle2d &circle);

    /* This expects symmetric matrix */
    static SecondOrderCurve FromMatrix(const Matrix33 &M);

    void prettyPrint();

    /* Operations with AST */
};


class Ellipse
{
public:
    Vector2dd center;  /**< center of the ellipse */
    Vector2dd axis;    /**< two main axis length  */
    double angle;      /**< angle of the first axis */

    /* https://en.wikipedia.org/wiki/Ellipse#In_analytic_geometry */
    SecondOrderCurve toSecondOrderCurve();
    static Ellipse FromPolinomial(const SecondOrderCurve &p);
    static Ellipse FromPolinomial(const double p[6]);

    Ellipse();
};

} // namespace corecvs

#endif // ELLIPSE_H
