/**
 * \file main_test_jit.cpp
 * \brief This is the main file for the test jit 
 *
 * \date сен 02, 2016
 * \author alexander
 *
 * \ingroup autotest  
 */

#include <iostream>
#include "gtest/gtest.h"
#include "homographyReconstructor.h"
#include "astNode.h"
#include "packedDerivative.h"

#include "global.h"


using namespace std;


double out17(double M3, double M4, double M5, double M6, double M7) {
return (
 (
  (
   (
    (
     (
      0.000000
     +
      (
       2.000000 * M3
      )
     )
    +
     (
      2.000000 * M4
     )

    )

   +
    (
     1.000000 * M5
    )

   )

  /
   (
    (
     (
      0.000000
     +
      (
       2.000000 * M6
      )

     )

    +
     (
      2.000000 * M7
     )

    )

   +
    (
     1.000000
    *
     1.000000
    )

   )

  )

 -
  2.316912
 )

);
}


TEST(jit, testjit)
{
    cout << "Starting test <jit>" << endl;
    cout << "This test is x64 and GCC only" << endl;

#if defined (__GNUC__) && __x86_64

    double sin_a, cos_a, a = 0.5;
    asm ("fldl %2;"
         "fsincos;"
         "fstpl %1;"
         "fstpl %0;" : "=m"(sin_a), "=m"(cos_a) : "m"(a));
    printf("sin(29°) = %f, cos(29°) = %f\n", sin_a, cos_a);
#endif


    HomographyReconstructor example;
    Matrix33 transform = Matrix33::RotateProj(degToRad(10));

    for (int i = 0; i < 3; i++ )
    {
        for (int j = 0; j < 3; j++ )
        {
            Vector2dd start(i,j);
            Vector2dd end = transform * start;
            example.addPoint2PointConstraint(start, end);
        }
    }

    Matrix33 exampleM = Matrix33::RotateProj(degToRad(-5.0)) * Matrix33::ShiftProj(0.4, 0.6);

    cout << "Example Matrix " << endl << exampleM << endl;

    /* Double run */
    {
        cout << "Double run.." << endl;
        double in[8] = {
            exampleM[0], exampleM[1], exampleM[2],
            exampleM[3], exampleM[4], exampleM[5],
            exampleM[6], exampleM[7]
        };
        double out[9*2];

        //TODO: this doesn't compile on win!!!
        //example.genericCostFunction<double>(in, out);
        for (size_t i = 0; i < CORE_COUNT_OF(out); i++)
        {
            cout << out[i] << endl;
        }
    }

    ASTContext::MAIN_CONTEXT = new ASTContext();

    /* AST run*/
    {
        cout << "AST run.." << endl;

        ASTNode in[8] = {
            ASTNode("M[0]"), ASTNode("M[1]"), ASTNode("M[2]"),
            ASTNode("M[3]"), ASTNode("M[4]"), ASTNode("M[5]"),
            ASTNode("M[6]"), ASTNode("M[7]")
        };
        ASTNode out[9*2];

        example.genericCostFunction<ASTNode>(in, out);

        std::map<std::string, double> binds = {
            {"M[0]", exampleM[0]}, {"M[1]", exampleM[1]}, {"M[2]", exampleM[2]},
            {"M[3]", exampleM[3]}, {"M[4]", exampleM[4]}, {"M[5]", exampleM[5]},
            {"M[6]", exampleM[6]}, {"M[7]", exampleM[7]}};

/*        for (size_t i = 0; i < CORE_COUNT_OF(out); i++)
        {
            out[i].p->codeGenCpp("out");
        }*/

        for (size_t i = 0; i < CORE_COUNT_OF(out); i++)
        {
            cout << out[i].p->compute(binds)->val << endl;
        }

        //out[17].p->codeGenCpp("out17");
        //out[17].p->compute(binds)->codeGenCpp("out17b");

        cout << out17(exampleM[3], exampleM[4], exampleM[5], exampleM[6], exampleM[7]) << endl;


    }

    #if 1
    /* Packed derivative run*/
    {
        cout << "PackedDerivative run.." << endl;
        PackedDerivative<8> in[8];
        for (size_t i = 0; i < CORE_COUNT_OF(in); i++ )
        {
            in[i] = PackedDerivative<8>::ID(exampleM[i], i);
        }
        PackedDerivative<8> out[9*2];

        example.genericCostFunction<PackedDerivative<8> >(in, out);

        for (size_t i = 0; i < CORE_COUNT_OF(out); i++)
        {
            cout << out[i] << endl;
        }
    }
#endif

    cout << "Test <jit> PASSED" << endl;
}
