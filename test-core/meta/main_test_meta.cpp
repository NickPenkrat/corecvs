/**
  * \file main_test_meta.cpp
 * \brief This is the main file for the test meta
 *
 * \date дек. 15, 2015
 * \author alexander
 *
 * \ingroup autotest
 */
#include <essentialEstimator.h>
#include <iostream>
#include "gtest/gtest.h"

#include "global.h"

#include "fixedVector.h"
#include "matrix33.h"
#include "quaternion.h"
#include "abstractBuffer.h"
#include "matrixOperations.h"

#include "packedDerivative.h"

#include "astNode.h"
#include "function.h"

using namespace std;
using namespace corecvs;

#if !defined(_WIN32) && !defined(_MSC_VER)

ASTNode operator "" _x(const char * name, size_t /*len*/)
{
    return ASTNode(name);
}

typedef GenericQuaternion<ASTNode> ASTQuaternion;



#if 0
template<class Element>
class AbsMatrix33 : public AbstractBuffer<Element, int>, public MatrixOperationsBase<AbsMatrix33<Element>, Element>
{
public:
    typedef Element ElementType;

    AbsMatrix33() : AbstractBuffer<Element, int>(3,3) {}

    /* Matrix Operations interface */
    Element &atm(int i, int j) {
        return this->element(i, j);
    }
    const Element &atm(int i, int j) const {
        return this->element(i, j);
    }

    static AbsMatrix33 createMatrix(int /*h*/, int /*w*/) {return AbsMatrix33(); }

    /* Additional helper function */
    void fillWithArgs(
             Element _a00, Element _a01, Element _a02,
             Element _a10, Element _a11, Element _a12,
             Element _a20, Element _a21, Element _a22
         )
    {
        this->element(0,0) = _a00;  this->element(0,1) = _a01;   this->element(0,2) = _a02;
        this->element(1,0) = _a10;  this->element(1,1) = _a11;   this->element(1,2) = _a12;
        this->element(2,0) = _a20;  this->element(2,1) = _a21;   this->element(2,2) = _a22;
    }
};
#endif

typedef AbsMatrixFixed<ASTNode, 3, 3> ASTMatrix33;
typedef AbsMatrixFixed<PackedDerivative<1>, 3, 3> PDMatrix33;

TEST(meta, testmeta)
{
    ASTContext::MAIN_CONTEXT = new ASTContext();

    cout << "Starting test <meta>" << endl;

    ASTNode e = (ASTNode("X") * (ASTNode(5.0)  + ASTNode(4.0)));
    e.p->codeGenCpp("test", e.p->identSymDefault);

    e.p->codeGenCpp("test", ASTNodeInt::identSymLine);

    vector<std::string> params;
    e.p->getVars(params);
    for (std::string p : params) {
        cout << p << ", ";
    }

    cout << "Some more advanced stuff" << endl;

    FixedVector<ASTNode, 5> test1;
    FixedVector<ASTNode, 5> test2;

    for (int i = 0; i < test1.LENGTH; i++)
    {
        test1[i] = ASTNode((double)i);
        test2[i] = ASTNode(1);
    }

    ASTNode dotProduct = (test1 & test2);
    ASTRenderDec dec = {"", "", true};
    dotProduct.p->codeGenCpp("dot_product", dec);

    ASTNodeInt *dotRes = dotProduct.p->compute();
    dotRes->codeGenCpp("dot_product_res", dec);


    cout << "Some more stuff" << endl;

    ASTQuaternion Q(  "Qx"_x     , ASTNode("Qy"), ASTNode("Qz"), ASTNode("Qt"));
    ASTQuaternion P(ASTNode("Px"), ASTNode("Py"), ASTNode("Pz"), ASTNode("Pt"));
    ASTQuaternion R(ASTNode("Rx"), ASTNode("Ry"), ASTNode("Rz"), ASTNode("Rt"));

    ASTNode Z = ((Q+(P^R)) & Q);
    Z.p->codeGenCpp("quaternion1", dec);

    ASTNodeInt *Zder = Z.p->derivative(std::string("Qx"));
    Zder->codeGenCpp("dQx", dec);

    ASTNodeInt *Zsimp = Zder->compute();
    Zsimp->codeGenCpp("dQx1", dec);

    ASTNodeInt *ZsimpAt7 = Zsimp->compute({{"Qx", 7.0}});
    ZsimpAt7->codeGenCpp("dQx7", dec);


    delete_safe(ASTContext::MAIN_CONTEXT);

    cout << "Test <meta> PASSED" << endl;
}


class Q2M : public FunctionArgs {
public:
    Q2M() : FunctionArgs(4, 9) {}

    virtual void operator()(const double in[], double out[])
    {
       Quaternion Q(in[0], in[1], in[2], in[3]);
       Matrix33 M = Q.toMatrix();
       for (int i = 0; i < M.H * M.W; i++)
           out[i] = M.element[i];

        /*for (int i = 0; i < 9; i++)
        {
            out[i] = (i < 4) ? in[i] : 1.0;
        }
        out[0] = in[0] * in[0];
        out[1] = sqrt(in[1]);*/
    }
};


TEST(meta, pdExample)
{
    typedef PackedDerivative<2> Derivative2D;

    {
        Derivative2D F = Derivative2D::X(5.0);
        std::cout << F << endl;
    }

    {
        Derivative2D B = Derivative2D::X(5.0);
        Derivative2D F = B * B;
        std::cout << F << endl;
    }

    {
        Derivative2D B = Derivative2D::X(5.0);
        Derivative2D F = sqrt(B);
        std::cout << F << endl;
    }
}

TEST(meta, matrixExample)
{
    ASTContext::MAIN_CONTEXT = new ASTContext();

    /* Symbol presentation */
    ASTQuaternion Q("x"_x, "y"_x, "z"_x, "t"_x);
    ASTMatrix33 M = Q.toMatrixGeneric<ASTMatrix33>();

    /*ASTMatrix33 M;
    M.atm(0, 0) = "x"_x * "x"_x;
    M.atm(0, 1) = sqrt("y"_x);
    M.atm(0, 2) = "z"_x;
    M.atm(1, 0) = "t"_x;
    M.atm(1, 1) = ASTNode(0.0);
    M.atm(1, 2) = ASTNode(0.0);
    M.atm(2, 0) = ASTNode(0.0);
    M.atm(2, 1) = ASTNode(0.0);
    M.atm(2, 2) = ASTNode(0.0);*/

    ASTRenderDec dec = {"", "", true};

    for (int i = 0; i < 1; i++)
    {
        for (int j = 1; j < 2; j++)
        {
            M.atm(i,j).p->codeGenCpp("matrixFromQ_00", dec);
            M.atm(i,j).p->derivative("y")->compute()->codeGenCpp("matrixFromQ_00", dec);
        }
    }
    /* Compiled presentation */

    Q2M func;


    /* Let's compare  */
    vector<std::string> names = {"x", "y", "z", "t"};
    vector<double> in = {0.2, 0.5, 0.1, 0.5};
    std::map<std::string, double> binds = {{"x", in[0]}, {"y", in[1]}, {"z", in[2]}, {"t", in[3]},};

    vector<double> out(9);
    vector<double> outs(9, numeric_limits<double>::max());

    func(&in[0], &out[0]);
    Matrix J = func.getJacobian(&in[0]);
    Matrix J1(J);

    cout << J << endl;

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            /* Value itself */
            ASTNodeInt *r = M.atm(i,j).p->compute(binds);
            if (r->op == ASTNodeInt::OPREATOR_NUM) {
                outs[i * 3 + j] = r->val;
            }

            cout <<  "value: " << out[i * 3 + j] << " "  << outs[i * 3 + j] << endl;

        }
    }

    //double maxdiff = 0;

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            for (size_t n = 0; n < names.size(); n++)
            {
                ASTNodeInt *rd = M.atm(i,j).p->derivative(names[n])->compute(binds);
                cout << rd->val << " ";

                J1.a(i*3+j, n) = fabs(rd->val - J.a(i*3+j, n));
            }
            cout << endl;
        }
    }

    cout << J1;


    /*Packed derivative*/
    GenericQuaternion<PackedDerivative<> > QP(PackedDerivative<>::X(in[0]), PackedDerivative<>(in[1]), PackedDerivative<>(in[2]), PackedDerivative<>(in[3]));
    PDMatrix33 MP = QP.toMatrixGeneric<PDMatrix33>();
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            cout << MP.atm(i,j) << endl;
        }
    }



    /*First compare the output */
}

TEST(meta, testmeta1)
{
    ASTContext::MAIN_CONTEXT = new ASTContext();

    std::cout << "Starting test <meta>" << std::endl;

    ASTNode e = (ASTNode("X") * (ASTNode(5.0)  + ASTNode(4.0)));
    ASTRenderDec dec = { "", "", true };

    e.p->codeGenCpp("test");
    e.p->codeGenCpp("test", dec);

    std::cout << "Some more advanced stuff" << std::endl;

    FixedVector<ASTNode, 5> test1;
    FixedVector<ASTNode, 5> test2;

    for (int i = 0; i < test1.LENGTH; i++)
    {
        test1[i] = ASTNode((double)i);
        test2[i] = ASTNode(1);
    }

    (test1 & test2).p->codeGenCpp("dot_product", dec);

    std::cout << "Some more stuff" << std::endl;

    GenericQuaternion<ASTNode> Q(ASTNode("Qx"), ASTNode("Qy"), ASTNode("Qz"), ASTNode("Qt"));
    GenericQuaternion<ASTNode> P(ASTNode("Px"), ASTNode("Py"), ASTNode("Pz"), ASTNode("Pt"));
    GenericQuaternion<ASTNode> R(ASTNode("Rx"), ASTNode("Ry"), ASTNode("Rz"), ASTNode("Rt"));
    ((Q+(P^R)) & Q).p->codeGenCpp("quaternion1", dec);

    delete_safe(ASTContext::MAIN_CONTEXT);

    std::cout << "Test <meta> PASSED" << std::endl;
}

#endif

#if !defined(_WIN32) && !defined(_MSC_VER)
TEST(meta, testMetaNodeFunction)
{
    cout << "Starting meta.testMetaNodeFunction" << endl;
    ASTContext::MAIN_CONTEXT = new ASTContext();
    ASTQuaternion Q(ASTNode("Qx"), ASTNode("Qy"), ASTNode("Qz"), ASTNode("Qt"));

    ASTMatrix33 M = Q.toMatrixGeneric<ASTMatrix33>();

    ASTNodeFunctionWrapper F;
    for (int i = 0; i < M.h; i++)
    {
        for (int j = 0; j < M.w; j++)
        {
            F.components.push_back(M.element(i,j).p);
        }
    }

    cout << F.getCCode();
}
#endif


TEST(meta, testEssentialCostFunction)
{

    double in[7] = {1/2, 1/2, -1/2, 1/2, 2, 2, 1};

    vector<Correspondence> data;
    for (int i = 0; i < 10; i++) {
        data.push_back(Correspondence(Vector2dd(i,i+10), Vector2dd(sqrt(i), -i)));
    }

    vector<Correspondence *> dataPtr;
    for (int i = 0; i < 10; i++) {
        dataPtr.push_back(&data[i]);
    }

    EssentialEstimator::CostFunction7toN     old   (&dataPtr);
    EssentialEstimator::CostFunction7toNPacked modern(&dataPtr);

    cout << "Old\n" << old.getJacobian(in) << endl;
    cout << "New\n" << modern.getJacobian(in) << endl;


}
