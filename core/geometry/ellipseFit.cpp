#include "ellipseFit.h"
#include "cholesky.h"
#include "cblasLapackeWrapper.h"

namespace corecvs {

EllipseFit::EllipseFit()
{

}

void EllipseFit::addPoint(Vector2dd &point)
{
    points.push_back(point);
}

/**
 *  This stuff uses a Cholesky decomposition to
 *  solve generalized eigenvalue problem
 *
 * \f[
 *    S a = l C a
 *    U U^T a = l C a
 *    l^(-1) U U^T a = C a
 *    l^(-1) U^T a  = U^(-1) C U^(-1)^T  U^T a
 *    l^(-1)(U^T a) = U^(-1) C U^(-1)^T (U^T a)
 * \f]
 *
 *
 **/
SecondOrderCurve EllipseFit::solveDummy()
{
    Matrix C(6,6,0.0);
    C.a(0,2) = 2;
    C.a(1,1) =-1;
    C.a(2,0) = 2;

    Matrix dD(points.size(), 6);
    for (vector<Vector2dd>::size_type i = 0; i < points.size(); i++) {
        Vector2dd p = points[i];
        dD.fillLineWithArgs(i, p.x() * p.x(), p.x() * p.y(), p.y() * p.y(), p.x(), p.y(), 1.0 );
    }

    Matrix S = dD.t() * dD;

    Matrix *U;
    Cholesky::uutDecompose(&S, &U);

    Matrix Right = U->inv() * C  * U->inv().t();

    Matrix A(Right);
    DiagonalMatrix D(A.h);
    Matrix V(A.h, A.w);
    Matrix::jacobi(&A, &D, &V, NULL);

    SecondOrderCurve result;

    for (int i = 0; i < 6; i++)
    {
        if (D.a(i) > 0) {
           // cout << "Positive eigen " << i << endl;
            Matrix rev =  U->t().inv() * V.column(i);

            result.a11() = rev.a(0,0);
            result.a22() = rev.a(2,0);
            result.a12() = rev.a(1,0) / 2;
            result.a13() = rev.a(3,0) / 2;
            result.a23() = rev.a(4,0) / 2;
            result.a33() = rev.a(5,0);
            break;
        }
    }

    return result;

}

#ifdef WITH_BLAS
SecondOrderCurve EllipseFit::solveBLAS()
{
    Matrix C(6,6,0.0);
    C.a(0,2) = 2;
    C.a(1,1) =-1;
    C.a(2,0) = 2;

    Matrix dD(points.size(), 6);
    for (vector<Vector2dd>::size_type i = 0; i < points.size(); i++) {
        Vector2dd p = points[i];
        dD.fillLineWithArgs(i, p.x() * p.x(), p.x() * p.y(), p.y() * p.y(), p.x(), p.y(), 1.0 );
    }

    Matrix S = dD.t() * dD;

    double er[6];
    double ei[6];
    double beta[6];

    Matrix VL(6, 6);
    Matrix VR(6, 6);

    cout << "VL.stride  :" << VL.stride << endl;
    cout << "VL.width   :" << VL.w      << endl;

    /**
         We should check alignment problems
     **/
    Matrix Sblas = S;
    Matrix Cblas = C;


    LAPACKE_dggev(LAPACK_ROW_MAJOR, 'N', 'V', 6, Sblas.data, 6, Cblas.data, 6, er, ei, beta, VL.data, 6, VR.data, 6);

    for (int i = 0; i < 6; i++)
    {
        cout << (er[i] / beta[i]) << endl;
    }

    cout << "VL" << endl << VL << endl;
    cout << "VR" << endl << VR << endl;

#ifdef DEBUG
    cout << "Sanity Check" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << "Checking vector: " << i << endl;
        //Matrix rev =  VR.row(i).t();
        Matrix rev =  VR.column(i);

        Matrix SR = S * rev;
        Matrix CR = C * rev;

        cout << "S * rev" << endl << SR << endl;
        cout << "C * rev" << endl << CR << endl;
        /*for (int j = 0; j < 6; j++)
            cout << "Sanity: " << SR.a(j,0) / CR.a(j,0) << endl;*/
        cout << "Sanity: " << (SR * beta[i] - CR * er[i]) << endl;
    }
#endif


    SecondOrderCurve result;

    for (int i = 0; i < 6; i++)
    {
        if (er[i] / beta[i] > 0) {
            cout << "Positive eigen " << i << endl;
            Matrix rev =  VR.column(i);
            cout << "Rev" << endl << rev << endl;

            result.a11() = rev.a(0,0);
            result.a22() = rev.a(2,0);
            result.a12() = rev.a(1,0) / 2;
            result.a13() = rev.a(3,0) / 2;
            result.a23() = rev.a(4,0) / 2;
            result.a33() = rev.a(5,0);
            break;
        }
    }

    result.prettyPrint();

    return result;

}
#endif

SecondOrderCurve EllipseFit::solveQuadricDummy()
{
    Matrix dD(points.size(), 6);
    for (vector<Vector2dd>::size_type i = 0; i < points.size(); i++) {
        Vector2dd p = points[i];
        dD.fillLineWithArgs(i, p.x() * p.x(), p.x() * p.y(), p.y() * p.y(), p.x(), p.y(), 1.0 );
    }

    Matrix S = dD.t() * dD;

    Matrix A(S);
    DiagonalMatrix D(A.h);
    Matrix V(A.h, A.w);
    Matrix::jacobi(&A, &D, &V, NULL);

    SecondOrderCurve result;


    int minnum = 0;
    double value = D.a(0);
    for (int i = 1; i < 6; i++)
    {
        if (D.a(i) < value) {
            minnum = i;
            value = D.a(i);
        }
    }

    Matrix rev = V.column(minnum);
    result.a11() = rev.a(0,0);
    result.a22() = rev.a(2,0);
    result.a12() = rev.a(1,0) / 2;
    result.a13() = rev.a(3,0) / 2;
    result.a23() = rev.a(4,0) / 2;
    result.a33() = rev.a(5,0);

    return result;
}

#if 0

/* Straitforward so far*/
Matrix dD(inputs.size(), 6);
for (vector<Vector2dd>::size_type i = 0; i < inputs.size(); i++) {
    Vector2dd p = inputs[i];
    dD.fillLineWithArgs(i, p.x() * p.x(), p.x() * p.y(), p.y() * p.y(), p.x(), p.y(), 1.0 );
}

Matrix S = dD.t() * dD;


Matrix *U;
Cholesky::uutDecompose(&S, &U);
cout << *U << endl;

cout << "Sanity Check0" << endl;
cout << (*U) * U->t() << endl;

// S a = l C a
// U U^T a = l C a
// l^(-1) U U^T a = C a
// l^(-1) U^T a  = U^(-1) C U^(-1)^T  U^T a
// l^(-1)(U^T a) = U^(-1) C U^(-1)^T (U^T a)

Matrix Right = U->inv() * C  * U->inv().t();

cout <<    "S" << endl <<    S << endl;
cout <<    "C" << endl <<    C << endl;
cout << "Right" << endl << Right << endl;

Matrix A(Right);
DiagonalMatrix D(A.h);
Matrix V(A.h, A.w);
Matrix::jacobi(&A, &D, &V, NULL);
cout << "Sanity Check1" << endl;
cout << V * D * V.t() << endl;

cout << "Sanity Check2"   << endl;
cout << "Eigen values:"   << endl << D << endl;
cout << "Eigen vectors0:" << endl << V << endl;

for (int i = 0; i < 6; i++)
{
    cout << "Check2.1" << endl;
    cout << (Right * V.column(i)).t() << endl;
    cout << "Check2.2" << endl;
    cout << (V.column(i) * D.a(i)).t() << endl;

    Matrix rev =  U->t().inv() * V.column(i);

    cout << "Check2.3" << endl;
    cout << (Right * U->t() * rev).t() << endl;
    cout << "Check2.4" << endl;
    cout << (U->t() * rev * D.a(i)).t() << endl;
    cout << "Check2.5" << endl;
    cout << (U->inv() * C  * U->inv().t() * U->t() * rev).t() << endl;
    cout << "Check2.6" << endl;
    cout << (U->inv() * C  * rev).t() << endl;

    cout << "Check3.1" << endl;
    cout << ((*U) * U->t() * rev * D.a(i)).t() << endl;
    cout << "Check3.2" << endl;
    cout << (C  * rev).t() << endl;
    cout << "Check3.3" << endl;
    cout << (S * rev * D.a(i) ).t() << endl;

}



cout << "Sanity Check" << endl;
cout << "UTI" << endl;
cout << U->t().inv();
cout << "UIT" << endl;
cout << U->inv().t();


for (int i = 0; i < 6; i++)
{
    cout << i << endl;
    Matrix column = U->t().inv() * V.column(i);
    Matrix L = (S * column).t();
    Matrix R = (C * column).t();
    cout << L << endl;
    cout << R << endl;
    for (int j = 0; j < 6; j++) {
        cout << (L.a(0,j) / R.a(0,j)) << " ";
    }
    cout << endl;
}



vector<double> result(6);
for (int i = 0; i < 6; i++)
{
    if (D.a(i) > 0) {
        cout << "Positive eigen " << i << endl;
        Matrix rev =  U->t().inv() * V.column(i);

        for (int j = 0; j < 6; j++) {
            result[j] = rev.a(j,0);
        }
        break;
    }
}

{
    double A = result[0];
    double B = result[1];
    double C = result[2];
    double D = result[3];
    double E = result[4];
    double F = result[5];

    cout << A << " " << B << " " << C << " " << D << " "<< E << " " << F << " " << endl;

    double Det = B * B - 4 * A * C;
    double T = (A * E * E + C * D * D - B * D * E + Det * F);
    double R = sqrt((A - C) * (A - C)  + B * B);
    double a = - sqrt(2 * T * (A + C + R)) / Det;
    double b = - sqrt(2 * T * (A + C - R)) / Det;

    double xc = (2 * C * D - B * E) / Det;
    double yc = (2 * A * E - B * D) / Det;
    double tetta = atan2(C - A - R, B);

    cout << "xc = " << xc << endl;
    cout << "yc = " << yc << endl;

    cout << "a = " << a << endl;
    cout << "b = " << b << endl;
    cout << "tetta = " << tetta << endl;

    int MAX = 850;
    for (int k = 0; k < MAX; k++)
    {
        double v = 2 * M_PI / MAX * k;
        Vector2dd point = Vector2dd(a * cos(v), b * sin(v));
        point = Matrix33::RotationZ(tetta) * point;
        point += Vector2dd(xc,yc);
        buffer->element(point.y(), point.x()) = RGBColor::Yellow();
    }

    for (size_t i = 0; i < inputs.size(); i++) {
        Vector2dd point = inputs[i];
        buffer->drawCrosshare3(point,RGBColor::Red());
    }

#endif

} // namespace corecvs

