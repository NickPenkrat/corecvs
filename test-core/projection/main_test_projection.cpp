/**
 * \file main_test_projection.cpp
 * \brief This is the main file for the test projection 
 *
 * \date янв 20, 2018
 * \author alexander
 *
 * \ingroup autotest  
 */

#include <iostream>
#include "gtest/gtest.h"

#include "core/cameracalibration/projection/equidistantProjection.h"
#include "core/cameracalibration/projection/equisolidAngleProjection.h"
#include "core/cameracalibration/projection/catadioptricProjection.h"
#include "core/utils/global.h"

using namespace std;
using namespace corecvs;


TEST(projection, testEquidistant)
{
    EquidistantProjection projection;

    Vector2dd source(Vector2dd(1.0, 1.0));
    Vector3dd p = projection.reverse(source);
    Vector2dd rsource = projection.project(p);

    cout << "EquidistantProjection:" << endl
         << projection << endl;
    cout << " Source: " << source << endl;
    cout << " RayDir: " << p << endl;
    cout << "RSource: " << rsource << endl;

    ASSERT_TRUE(source.notTooFar(rsource, 1e-7));
}


TEST(projection, testEquisolid)
{
    EquisolidAngleProjection projection;

    Vector2dd source(Vector2dd(1.0, 1.0));
    Vector3dd p = projection.reverse(source);
    Vector2dd rsource = projection.project(p);

    cout << "EquisolidAngleProjection:" << endl
         << projection << endl;
    cout << " Source: " << source << endl;
    cout << " RayDir: " << p << endl;
    cout << "RSource: " << rsource << endl;

    ASSERT_TRUE(source.notTooFar(rsource, 1e-7));
}

TEST(projection, testCatadioptric)
{
    CatadioptricProjection projection;

    Vector2dd source(Vector2dd(1.0, 1.0));
    Vector3dd p = projection.reverse(source);
    Vector2dd rsource = projection.project(p);

    cout << "CatadioptricBaseParameters:" << endl
         << projection << endl;
    cout << " Source: " << source << endl;
    cout << " RayDir: " << p << endl;
    cout << "RSource: " << rsource << endl;

    ASSERT_TRUE(source.notTooFar(rsource, 1e-7));
}
