#include "matrix22.h"
#include "ellipse.h"

namespace corecvs {





SecondOrderCurve Ellipse::toSecondOrderCurve()
{

    SecondOrderCurve p;
    double sa = sin(angle);
    double ca = cos(angle);

    double a2 = axis.x() * axis.x();
    double b2 = axis.y() * axis.y();

    p[0] = a2 * sa * sa + b2 * ca * ca;
    p[1] = 2 * (b2 - a2) * sa * ca;
    p[2] = a2 * ca * ca + b2 * sa * sa;
    p[3] = -2 * p[0] * center.x() - p[1] * center.y();
    p[4] = -p[1] * center.x() - 2 * p[2] * center.y();
    p[5] = p[0] * center.x() * center.x() + p[1] * center.x() * center.y() + p[2] * center.y() * center.y() - a2*b2;

    return p;
}

Ellipse Ellipse::FromPolinomial(const SecondOrderCurve &p)
{
    return FromPolinomial(&p[0]);
}

Ellipse Ellipse::FromPolinomial(const double p[])
{
    double A = p[0];
    double B = p[1];
    double C = p[2];
    double D = p[3];
    double E = p[4];
    double F = p[5];

    cout << A << " " << B << " " << C << " " << D << " "<< E << " " << F << " " << std::endl;

    double Det = B * B - 4 * A * C;
    double T = (A * E * E + C * D * D - B * D * E + Det * F);
    double R = sqrt((A - C) * (A - C)  + B * B);
    double a = - sqrt(2 * T * (A + C + R)) / Det;
    double b = - sqrt(2 * T * (A + C - R)) / Det;

    double xc = (2 * C * D - B * E) / Det;
    double yc = (2 * A * E - B * D) / Det;
    double tetta = atan2(C - A - R, B);

#if DEBUG
    cout << "xc = " << xc << endl;
    cout << "yc = " << yc << endl;

    cout << "a = " << a << endl;
    cout << "b = " << b << endl;
    cout << "tetta = " << tetta << endl;
#endif

    Ellipse result;
    result.center = Vector2dd(xc, yc);
    result.axis   = Vector2dd( a,  b);
    result.angle = tetta;
    return result;
}

Ellipse::Ellipse()
{

}

Matrix33 SecondOrderCurve::toMatrixForm() const
{
    return Matrix33(
        a11(), a12(), a13(),
        a21(), a22(), a23(),
        a31(), a32(), a33()
                );
}

SecondOrderCurve SecondOrderCurve::FromMatrix(const Matrix33 &M)
{
    SecondOrderCurve result;
    result.a11() = M.a(0,0);
    result.a12() = M.a(0,1);
    result.a22() = M.a(1,1);
    result.a13() = M.a(0,2);
    result.a23() = M.a(1,2);
    result.a33() = M.a(2,2);

    return result;
}


/**

  x^T M x = 0
  x -> B x = y

  (B x)^T B^T^(-1) M B^(-1) (B x) = 0


  ( b11 b21  0 ) -1 ( a11 a12 a13 )  ( b11 b12  b_x )
  ( b12 b22  0 )    ( a12 a22 a23 )  ( b21 b22  b_y )  = 0
  ( b_x b_y  1 )    ( a13 a23 a33 )  (  0   0    1  )

 **/

void SecondOrderCurve::transform(const Matrix33 &B)
{
    (*this) = SecondOrderCurve::FromMatrix(B.t().inv() * toMatrixForm() * B.inv());
}

SecondOrderCurve SecondOrderCurve::transformed(const Matrix33 &B)
{
    return SecondOrderCurve::FromMatrix(B.t().inv() * toMatrixForm() * B.inv());
}

double SecondOrderCurve::invDelta() const
{
    return toMatrixForm().det();
}

double SecondOrderCurve::invD() const
{
    Matrix22 minor = Matrix22(
            a11(), a12(),
            a21(), a22()
    );
    cout <<  "Minor" << minor << endl;

    return minor.det();
}

double SecondOrderCurve::invI() const
{
    return a11() + a22();
}

SecondOrderCurve::CurveClass SecondOrderCurve::classify(double epsilon)
{
    double deltaInv = invDelta();
    if (fabs(deltaInv) < epsilon ) {
        return DEGENERATE;
    } else {
        double dInv = invD();
        if (fabs(dInv) < epsilon ) {
            return PARABOLA;
        }

        if (dInv < 0) {
            return HYPERBOLA;
        } else {
            double iInv = invI();
            if (iInv * deltaInv < 0)
            {
                if (fabs(iInv * iInv - 4 * dInv) < epsilon) {
                    return CIRCLE;
                } else {
                    return ELLIPSE;
                }
            }
            else
                return IMAGINARTY_ELLIPSE;
        }
    }
    return DEGENERATE;
}

Vector2dd SecondOrderCurve::tangent(Vector2dd &point) const
{
    return Vector2dd(
        a11() * point.x() +  a12() * point.y() + a13(),
        a12() * point.x() +  a22() * point.y() + a23()
                );
}

double SecondOrderCurve::value(const Vector2dd &p) const
{

    return  value(p.x(), p.y());
}

double SecondOrderCurve::value(double x, double y) const
{
    return       a11() * x * x + a22() * y * y +
           2 * ( a12() * x * y +
                 a13() * x + a23() * y )  +
                 a33();
}

bool SecondOrderCurve::isInside(double x, double y) const
{
    return value(x, y) < 0;
}

bool SecondOrderCurve::isInside(const Vector2dd &point) const
{
    return isInside(point.x(), point.y());
}

/**
  \f[ (x - x_0)^2 + (y - y_0)^2 = R^2 \f]
  \f[ (x^2 - 2 * x_0 * x + x_0^2) + (y^2 - 2 * y_0 * x + y_0^2) = R^2 \f]


 **/
SecondOrderCurve SecondOrderCurve::FromCircle(const Vector2dd &center, double r)
{
    SecondOrderCurve circle;
    circle.a11() = 1.0; // xx
    circle.a12() = 0.0; // xy
    circle.a22() = 1.0; // yy

    circle.a13() = -center.x(); // x
    circle.a23() = -center.y(); // y

    circle.a33() = center.sumAllElementsSq() - r * r; // free
    return circle;
}

SecondOrderCurve SecondOrderCurve::FromCircle(const Circle2d &circle)
{
    return FromCircle(circle.c, circle.r);
}


void SecondOrderCurve::prettyPrint()
{
    cout << a11() << "x^2 + " << a22() << "y^2 + 2 *" << a12() << "xy + 2 *" << a13() << "x + 2 * " << a23() << "y +" << a33() << endl;
    cout << getName(classify()) << endl;
    cout << "Delta :" << invDelta()  << endl;
    cout << "D     :" << invD()      << endl;
    cout << "I     :" << invI()      << endl;

}



} // namespace corecvs
