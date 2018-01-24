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

#include "core/cameracalibration/cameraModel.h"
#include "core/utils/global.h"



#include "core/buffers/rgb24/rgb24Buffer.h"
#include "core/geometry/mesh3d.h"
#include "core/cameracalibration/calibrationDrawHelpers.h"
#include "core/fileformats/bmpLoader.h"


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

    cout << "Will call project" << endl;
    Vector2dd rsource = projection.project(p);

    cout << "CatadioptricBaseParameters:" << endl
         << projection << endl;
    cout << " Source: " << source << endl;
    cout << " RayDir: " << p << endl;
    cout << "RSource: " << rsource << endl;

    ASSERT_TRUE(source.notTooFar(rsource, 1e-7));
}

TEST(projection, testCatadioptric1)
{
    CatadioptricProjection projection;

    projection.mN[0] = 0.1;

    Vector2dd source(Vector2dd(1.0, 1.0));
    Vector3dd p = projection.reverse(source);

    cout << "Will call project" << endl;
    Vector2dd rsource = projection.project(p);

    cout << "CatadioptricBaseParameters:" << endl
         << projection << endl;
    cout << " Source: " << source << endl;
    cout << " RayDir: " << p << endl;
    cout << "RSource: " << rsource << endl;
}

TEST(projection, testCatadioptric2)
{
    CatadioptricProjection projection;

    projection.mN[0] = 0.1;
    projection.mN[1] = 0.2;

    Vector2dd source(Vector2dd(1.0, 1.0));
    Vector3dd p = projection.reverse(source);

    cout << "Will call project" << endl;
    Vector2dd rsource = projection.project(p);

    cout << "CatadioptricBaseParameters:" << endl
         << projection << endl;
    cout << " Source: " << source << endl;
    cout << " RayDir: " << p << endl;
    cout << "RSource: " << rsource << endl;

    ASSERT_TRUE(source.notTooFar(rsource, 1e-7));
}

TEST(projection, testCatadioptric3)
{
    CatadioptricProjection projection;

    projection.mN = vector<double>({ -3.97282, -7.61552, 26.471, -43.6205});

    Vector2dd source(Vector2dd(1.0, 1.0));
    Vector3dd p = projection.reverse(source);

    cout << "Will call project" << endl;
    Vector2dd rsource = projection.project(p);

    cout << "CatadioptricBaseParameters:" << endl
         << projection << endl;
    cout << " Source: " << source << endl;
    cout << " RayDir: " << p << endl;
    cout << "RSource: " << rsource << endl;

    ASSERT_TRUE(source.notTooFar(rsource, 1e-7));
}

TEST(projection, testFormatLoad)
{
    std::string input =
    "omnidirectional\n"
    "1578 1.35292 1.12018 5 0.520776 -0.561115 -0.560149 1.01397 -0.870155";
    std::istringstream ss(input);

    CameraModel model = CameraModel::loadCatadioptricFromTxt(ss);
    cout << model;
}


TEST(projection, testProjectionChange)
{
    CatadioptricProjection slowProjection;

    // RGB24Buffer *image = BufferFactory::getInstance()->loadG12Bitmap("data/pair/image0001_c0.pgm");

    int h = 480;
    int w = 640;

    RGB24Buffer *image = new RGB24Buffer(h,w);

    slowProjection.setSizeX(w);
    slowProjection.setSizeY(h);

    slowProjection.setPrincipalX(image->w / 2.0);
    slowProjection.setPrincipalY(image->h / 2.0);

    slowProjection.setFocal(image->w / 2.0);

    CameraModel model(slowProjection.clone());

    Mesh3D mesh;
    mesh.switchColor(true);
    mesh.setColor(RGBColor::Yellow());
    CalibrationDrawHelpers draw;
    draw.drawCamera(mesh, model, 5);
    mesh.dumpPLY("catadioptric.ply");


    Mesh3D toDraw;
    toDraw.addIcoSphere(Vector3dd(0, 0, 100.0), 10, 2);

    for (size_t i = 0; i < toDraw.vertexes.size(); i++)
    {
        Vector2dd prj = model.project(toDraw.vertexes[i]);
        Vector2d<int> prji(fround(prj.x()), fround(prj.y()));

        if (image->isValidCoord(prji))
        image->element(prji) = RGBColor::Red();
    }

    BMPLoader().save("catad.bmp", image);


}
