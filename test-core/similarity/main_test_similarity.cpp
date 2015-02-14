/**
 * \file main_test_similarity.cpp
 * \brief This is the main file for the test similarity 
 *
 * \date февр. 13, 2015
 * \author alexander
 *
 * \ingroup autotest  
 */

#include <iostream>
#include <fstream>
#include <vector>

#include "global.h"
#include "vector3d.h"
#include "similarityReconstructor.h"
#include "mesh3d.h"

using namespace std;
using namespace corecvs;


void testSimilarity()
{
    Matrix44 transform = Quaternion::Rotation(Vector3dd(0.345,-0.2,-123), 0.12).toMatrix() * Matrix44::Scale(2.0);

    vector<Vector3dd> data;
    data.push_back(Vector3dd(0,0,0));
    data.push_back(Vector3dd(1,0,0));
    data.push_back(Vector3dd(1,1,0));
    data.push_back(Vector3dd(0,1,0));

    vector<Vector3dd> out;
    for (unsigned i = 0; i < data.size(); i++)
    {
        Vector3dd noise(randRanged<double>(1.0,-1.0), randRanged<double>(1.0,-1.0), randRanged<double>(1.0,-1.0));
        out.push_back(transform * data[i] + noise * 0.2);
    }

    SimilarityReconstructor reconstructor;


    for (unsigned i = 0; i < data.size(); i++)
    {
        reconstructor.addPoint2PointConstraint(data[i], out[i]);
    }

    Similarity sim = reconstructor.getBestSimilarity();

    cout << sim << endl;

    Matrix44 m = sim.toMatrix();

    Mesh3D mesh;
    for (unsigned i = 0; i < data.size(); i++)
    {

        cout << "(" << ((out[i].notTooFar(m * data[i], 1e-7)) ? "V" : "X") << ")  "
             << data[i] << " => " << m * data[i] << " =? " << out[i] <<  endl;

        mesh.addPoint(data[i]);
        mesh.addPoint(out[i]);
        mesh.addPoint(m * data[i]);
    }

    double cost = reconstructor.getCostFunction(sim);
    cout << "Result:" << cost * cost << endl;

    Similarity sim1 = reconstructor.getBestSimilarityLM(sim);
    cost = reconstructor.getCostFunction(sim1);
    cout << "Result LM:" << cost * cost  << endl;

    Similarity sim2 = reconstructor.getBestSimilarityLM(sim);
    cost = reconstructor.getCostFunction(sim2);
    cout << "Result LM(N):" << cost * cost  << endl;



    std::fstream file;
    file.open("out.ply", std::ios::out);
    mesh.dumpPLY(file);
    file.close();

}

void testCostFunction (void)
{
    Similarity s;
    s.rotation = Quaternion::Rotation(Vector3dd(1.0,2.0,3.0), 4.0);
    s.scaleL = 5.0;
    s.scaleR = 6.0;

    s.shiftL = Vector3dd( 7.0,  8.0,  9.0);
    s.shiftR = Vector3dd(10.0, 11.0, 12.0);

    Matrix44 matrix = s.toMatrix();

    double in[Similarity::PARAM_NUMBER];
    s.fillFunctionInput(in);

    Similarity s1(in);
    Matrix44 matrix1 = s1.toMatrix();

    for(int i = 0; i < Similarity::PARAM_NUMBER; i++)
    {
        cout << in[i] << " ";
    }
    cout << endl;
    cout << "=======" << endl;
    cout << s << endl;
    cout << "=======" << endl;
    cout << s1 << endl;


    cout << "=======" << endl;
    cout << matrix << endl;
    cout << "=======" << endl;
    cout << matrix1 << endl;

}

int main (int /*argC*/, char **/*argV*/)
{
    testSimilarity();
    //testCostFunction();

    cout << "PASSED" << endl;
    return 0;
}
