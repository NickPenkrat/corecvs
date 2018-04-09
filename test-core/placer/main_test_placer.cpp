/**
 * \file main_test_placer.cpp
 * \brief This is the main file for the test placer 
 *
 * \date фев 22, 2018
 * \author alexander
 *
 * \ingroup autotest  
 */

#include <iostream>
#include "gtest/gtest.h"

#include "core/utils/global.h"
#include "core/placer/extrinsicsPlacer.h"

using namespace std;
using namespace corecvs;


TEST(placer, testplacer)
{
    SimplifiedScene scene;
    scene.points.resize(2);
    scene.cameras.resize(2);

    for (size_t i = 0; i < scene.points.size(); i++)
    {
        scene.points[i] = Vector3dd(i + 0.1, i + 0.1, i + 0.1);
    }

    scene.obs.push_back(SimplifiedScene::Observation(0,0, Vector3dd::Zero()));
    scene.obs.push_back(SimplifiedScene::Observation(1,1, Vector3dd::Zero()));
    scene.obs.push_back(SimplifiedScene::Observation(2,2, Vector3dd::Zero()));
    scene.obs.push_back(SimplifiedScene::Observation(3,3, Vector3dd::Zero()));

    ExtrinsicsPlacerParameters params;
    params.setLock1Cam(true);
    SillyCostMask cost(&scene, params);
    vector<double> m = cost.sceneToModel(scene);
    cout << "Model vector:" << m << endl;

    vector<double> p(cost.outputs);
    cost.operator ()(m, p);
    cost.sceneFromModel(scene, m);
}
