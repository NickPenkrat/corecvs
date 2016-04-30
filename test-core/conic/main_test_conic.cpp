/**
 * \file main_test_conic.cpp
 * \brief This is the main file for the test conic 
 *
 * \date сент. 29, 2015
 * \author alexander
 *
 * \ingroup autotest  
 */

#include <iostream>
#include "gtest/gtest.h"

#include "global.h"

#include "vector2d.h"
#include "conic.h"
#include "rgb24Buffer.h"
#include "bmpLoader.h"
#include "abstractPainter.h"

#include "mesh3d.h"

using namespace std;
using namespace corecvs;

TEST(conic, testCircleHasPoint)
{
    Circle2d c (0, 0, 5);

    ASSERT_TRUE (c.hasPoint(Vector2dd(0,5)));
    ASSERT_TRUE (c.hasPoint(Vector2dd(5,0)));
    ASSERT_TRUE (c.hasPoint(Vector2dd(-3,4)));
    ASSERT_FALSE(c.hasPoint(Vector2dd(-7,4)));
}

TEST(conic, testCircleIntersection)
{

    Circle2d c1(0, 0, 3);
    Circle2d c2(5, 0, 4);

    Vector2dd p1(0.0, 0.0);
    Vector2dd p2(0.0, 0.0);

    ASSERT_TRUE(c1.intersectWith(c2, p1, p2));
    cout << "I1:" << p1 << endl;
    cout << "I2:" << p2 << endl;
    ASSERT_TRUE (c1.hasPoint(p1));
    ASSERT_TRUE (c1.hasPoint(p2));
    ASSERT_TRUE (c2.hasPoint(p1));
    ASSERT_TRUE (c2.hasPoint(p2));

    Circle2d c3(7.5, 0, 4);
    ASSERT_FALSE(c3.intersectWith(c1, p1, p2));
    ASSERT_FALSE(c1.intersectWith(c3, p1, p2));

    ASSERT_TRUE (c3.intersectWith(c2, p1, p2));
    ASSERT_TRUE (c2.intersectWith(c3, p1, p2));


    Circle2d c4(1, 0, 1.49);
    ASSERT_FALSE(c4.intersectWith(c1, p1, p2));
    ASSERT_FALSE(c1.intersectWith(c4, p1, p2));

    cout << "Test <conic> PASSED" << endl;
}

TEST(conic, testCircleIntersection1)
{

    Circle2d c1( 60,  60, 20 * sqrt(10.0));
    Circle2d c2(140, 140, 20 * sqrt(10.0));

    Vector2dd p1(0.0, 0.0);
    Vector2dd p2(0.0, 0.0);

    ASSERT_TRUE(c1.intersectWith(c2, p1, p2));
    cout << "I1:" << p1 << endl;
    cout << "I2:" << p2 << endl;
    ASSERT_TRUE (c1.hasPoint(p1));
    ASSERT_TRUE (c1.hasPoint(p2));
    ASSERT_TRUE (c2.hasPoint(p1));
    ASSERT_TRUE (c2.hasPoint(p2));

    /**/
    RGB24Buffer drawing(220,220);

    //    AbstractPainter<RGB24Buffer>(&drawing).drawCircle(c1, RGBColor::Red  ());
    //    AbstractPainter<RGB24Buffer>(&drawing).drawCircle(c2, RGBColor::Green());
    drawing.drawArc(c1, RGBColor::Red  ());
    drawing.drawArc(c2, RGBColor::Green());
    drawing.setElement(fround(p1.y()), fround(p1.x()), RGBColor::Blue());
    drawing.setElement(fround(p2.y()), fround(p2.x()), RGBColor::Blue());
    BMPLoader().save("circles-int.bmp", &drawing);
    /**/

}

TEST(conic, testSphereIntersection)
{
    Sphere3d s1( 0.0,  0.0,  0.0, 20.0);
    Sphere3d s2(10.0, 20.0, 20.0, 20.0);

    Circle3d c1;
    ASSERT_TRUE(s1.intersectWith(s2, c1));

    for (int i = 0; i < 8; i++) {
        ASSERT_TRUE(s1.hasPoint(c1.getPoint(degToRad(i * 45.0))));
        ASSERT_TRUE(s2.hasPoint(c1.getPoint(degToRad(i * 45.0))));
    }

    Sphere3d s3( 100.0,  100.0,  0.0, 20.0);
    Circle3d c2;
    ASSERT_FALSE(s1.intersectWith(s3, c2));


    Mesh3D mesh;
    mesh.switchColor();
    mesh.setColor(RGBColor::Red());
    mesh.addIcoSphere(s1);

    mesh.setColor(RGBColor::Green());
    mesh.addIcoSphere(s2);

    mesh.setColor(RGBColor::Blue());
    mesh.addCircle(c1);

    mesh.dumpPLY("sphere-int.ply");
}
