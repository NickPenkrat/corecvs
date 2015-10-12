/**
 * \file main_test_meshdraw.cpp
 * \brief This is the main file for the test meshdraw 
 *
 * \date окт. 11, 2015
 * \author alexander
 *
 * \ingroup autotest  
 */

#include <iostream>
#include "gtest/gtest.h"

#include "global.h"
#include "mesh3d.h"

using namespace std;
using namespace corecvs;


TEST(meshdraw, testmeshdraw)
{
    cout << "Starting test <meshdraw>. This is a manual test" << endl;
    Mesh3D mesh;
    mesh.switchColor();
    mesh.addOrts(10, true);
    mesh.dumpPLY("mesh-orts.ply");
}
