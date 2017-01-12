/**
 * \file main_test_polygon.cpp
 * \brief This is the main file for the test polygon 
 *
 * \date дек 27, 2016
 * \author alexander
 *
 * \ingroup autotest  
 */

#include <iostream>
#include "gtest/gtest.h"

#include "polygons.h"
#include "global.h"
#include "mathUtils.h"
#include "rgb24Buffer.h"
#include "bmpLoader.h"
#include "abstractPainter.h"

#include "calibrationCamera.h"
#include "raytraceRenderer.h"
#include "raytraceObjects.h"
#include "calibrationHelpers.h"


using namespace std;
using namespace corecvs;

TEST(polygon, testArea1)
{
    cout << "Starting test <polygon>" << endl;

    Polygon  pentakle = Polygon::RegularPolygon(5, Vector2dd::Zero(), 10);
    Polygon rpentakle = Polygon::Reverse(pentakle);

    cout << "Center :" <<  pentakle.center() << endl;
    cout << "RCenter:" << rpentakle.center() << endl;

    CORE_ASSERT_TRUE_P( pentakle.center().notTooFar(Vector2dd::Zero(), 1e-8), ("Wrong Center"));
    CORE_ASSERT_TRUE_P(rpentakle.center().notTooFar(Vector2dd::Zero(), 1e-8), ("Wrong Center of reversed"));

    double sarea1 =  pentakle.signedArea();
    double sarea2 = rpentakle.signedArea();
    double realArea = 5.0/2.0 * (10.0 * 10.0) * sin(2.0/5.0 * M_PI);

    cout << "True Area:" << realArea << endl;
    cout << "     Area:" << sarea1 << endl;
    cout << "Rev  Area:" << sarea2 << endl;


    CORE_ASSERT_DOUBLE_EQUAL_EP(sarea1, -realArea, 1e-8, ("Wrong Area"));
    CORE_ASSERT_DOUBLE_EQUAL_EP(sarea2, realArea, 1e-8, ("Wrong Area of reversed"));

    cout << "Test <polygon> PASSED" << endl;
}


/**
 * Try to test  non convex polygon
 *
 *
 *    **  **
 *    **  **
 *    **  **
 *    **  **
 *    ******
 *    ******
 **/
TEST(polygon, testArea2)
{
    cout << "Starting test <polygon>" << endl;

    Polygon  p;
    p.push_back(Vector2dd(0.0, 0.0));
    p.push_back(Vector2dd(2.0, 0.0));
    p.push_back(Vector2dd(2.0, 4.0));
    p.push_back(Vector2dd(4.0, 4.0));
    p.push_back(Vector2dd(4.0, 0.0));
    p.push_back(Vector2dd(6.0, 0.0));
    p.push_back(Vector2dd(6.0, 6.0));
    p.push_back(Vector2dd(0.0, 6.0));

    Polygon rp = Polygon::Reverse(p);

    cout << "Center :" <<  p.center() << endl;
    cout << "RCenter:" << rp.center() << endl;

    CORE_ASSERT_TRUE_P( p.center().notTooFar(Vector2dd(3, 2.5), 1e-8), ("Wrong Center"));
    CORE_ASSERT_TRUE_P(rp.center().notTooFar(Vector2dd(3, 2.5), 1e-8), ("Wrong Center of reversed"));

    double sarea1 =  p.signedArea();
    double sarea2 = rp.signedArea();
    double realArea = (6*6) - 2 * 4;

    cout << "True Area:" << realArea << endl;
    cout << "     Area:" << sarea1 << endl;
    cout << "Rev  Area:" << sarea2 << endl;


    CORE_ASSERT_DOUBLE_EQUAL_EP(sarea1, -realArea, 1e-8, ("Wrong Area"));
    CORE_ASSERT_DOUBLE_EQUAL_EP(sarea2, realArea, 1e-8, ("Wrong Area of reversed"));

    cout << "Test <polygon> PASSED" << endl;
}


TEST(polygon, testGiftWrap)
{
    Polygon  p;
    p.push_back(Vector2dd(0.0, 0.0));
    p.push_back(Vector2dd(2.0, 0.0));
    p.push_back(Vector2dd(2.0, 4.0));
    p.push_back(Vector2dd(4.0, 4.0));
    p.push_back(Vector2dd(4.0, 0.0));
    p.push_back(Vector2dd(6.0, 0.0));
    p.push_back(Vector2dd(6.0, 6.0));
    p.push_back(Vector2dd(0.0, 6.0));

    Polygon result = ConvexHull::GiftWrap(p);
    cout << result << endl;

    for (size_t i = 0; i < result.size(); i++)
    {
        Vector2dd point = result[i];
        bool perefery = !point.isInRect(Vector2dd(0.01,0.01), Vector2dd(5.99,5.99));
        CORE_ASSERT_TRUE(perefery, "Convex hull is wrong");
    }
}

TEST(polygon, testRayIntersection)
{

    Ray2d r1 = Ray2d::FromPoints(Vector2dd(5.0,  5.0), Vector2dd(95.0, 95.0));
    Ray2d r2 = Ray2d::FromPoints(Vector2dd(5.0, 95.0), Vector2dd(95.0, 5.0));

    cout << "Ray 1:" << r1 << endl;
    cout << "Ray 2:" << r2 << endl;

    bool hasInt = Ray2d::hasIntersection(r1, r2);
    cout << "Has intersection" << hasInt << endl;
    CORE_ASSERT_TRUE(hasInt, "Intersection is wrong");

    double t1, t2;
    Vector2dd x = Ray2d::intersection(r1, r2, t1, t2);
    cout << "t1:" << t1 << endl;
    cout << "t2:" << t2 << endl;
    cout << "X :" << x << endl;

    CORE_ASSERT_DOUBLE_EQUAL(t1, 0.5, "Wrong intersection");
    CORE_ASSERT_DOUBLE_EQUAL(t2, 0.5, "Wrong intersection");
    CORE_ASSERT_TRUE(x.notTooFar(Vector2dd(50.0, 50.0)), "Intersection corrdinates are wrong");

}

TEST(polygon, testRayNoIntersection)
{
    Ray2d r1 = Ray2d::FromPointAndDirection(Vector2dd( 10.0, 10.0) , Vector2dd(100.0, 0.0));
    Ray2d r2 = Ray2d::FromPointAndDirection(Vector2dd(110.0, 110.0), Vector2dd(150.0, 0.0));

    cout << "Ray 1:" << r1 << endl;
    cout << "Ray 2:" << r2 << endl;

    bool hasInt = Ray2d::hasIntersection(r1, r2);
    cout << "Has intersection:" << hasInt << endl;
    CORE_ASSERT_FALSE(hasInt, "There should be no intersection");

}

/* Some visual check*/
TEST(polygon, testRayIntersectionVisual)
{
    int h = 2000;
    int w = 2000;
    int sample = 40;

    std::mt19937 rng;
    std::uniform_real_distribution<double> runif(2, sample - 2);

    RGB24Buffer *buffer  = new RGB24Buffer(h, w, RGBColor::Black());

    for (int i = 0; i < (h / sample); i++)
        for (int j = 0; j < (w / sample); j++)
        {
            Vector2dd shift(i * sample, j*sample);

            Vector2dd a(runif(rng), runif(rng));
            Vector2dd b(runif(rng), runif(rng));
            Vector2dd a1(runif(rng), runif(rng));
            Vector2dd b1(runif(rng), runif(rng));
            Ray2d r1 = Ray2d::FromPoints(a,a1);
            Ray2d r2 = Ray2d::FromPoints(b,b1);

            double t1, t2;
            Vector2dd x = Ray2d::intersection(r1, r2, t1, t2);

            buffer->drawLine(r1.getStart() + shift, r1.getEnd() + shift, RGBColor::Red());
            buffer->drawLine(r2.getStart() + shift, r2.getEnd() + shift, RGBColor::Blue());

            if (t1 != std::numeric_limits<double>::infinity() && x.isInRect(Vector2dd(0,0), Vector2dd(sample,sample)))
                buffer->drawCrosshare1(x + shift, RGBColor::Green());


        }

    BMPLoader().save("inetrsection.bmp", buffer);
    delete_safe(buffer);

}

TEST(polygon, testWindingNumber)
{
    int h = 200;
    int w = 200;

    Polygon  p;
    p.push_back(Vector2dd(0.0, 0.0));
    p.push_back(Vector2dd(2.0, 0.0));
    p.push_back(Vector2dd(2.0, 4.0));
    p.push_back(Vector2dd(4.0, 4.0));
    p.push_back(Vector2dd(4.0, 0.0));
    p.push_back(Vector2dd(6.0, 0.0));
    p.push_back(Vector2dd(6.0, 1.0));
    p.push_back(Vector2dd(1.0, 2.0));
    p.push_back(Vector2dd(6.0, 3.0));
    p.push_back(Vector2dd(6.0, 6.0));
    p.push_back(Vector2dd(0.0, 6.0));

    p.transform(Matrix33::ShiftProj(10,10) * Matrix33::Scale2(30));

    RGB24Buffer *buffer  = new RGB24Buffer(h, w, RGBColor::Black());
    AbstractPainter<RGB24Buffer> painter(buffer);

    RGBColor palette[] =
    {
        RGBColor(0x762a83u),
        RGBColor(0xaf8dc3u),
        RGBColor(0xe7d4e8u),
        RGBColor(0xd9f0d3u),
        RGBColor(0x7fbf7bu),
        RGBColor(0x1b7837u)
    };


    for (int i = 0; i < h ; i++)
    {
        for (int j = 0; j < w ; j++)
        {
            int wn = p.windingNumber(Vector2dd(j,i));
            int color = wn + 2;
            if (color < 0) color = 0;
            if (color >= (int)CORE_COUNT_OF(palette)) color = CORE_COUNT_OF(palette) - 1;
            buffer->element(i, j)  = palette[color];
        }
    }

    painter.drawPolygon(p, RGBColor::Blue());

    BMPLoader().save("winding.bmp", buffer);
    delete_safe(buffer);

}


TEST(polygon, testIntersection)
{
    int h = 400;
    int w = 400;

    Polygon  p1 = Polygon::RegularPolygon(5, Vector2dd(w / 2 , h / 2), 180);
    Polygon  p2 = Polygon::Reverse(Polygon::RegularPolygon(5, Vector2dd(w / 2 , h / 2), 180, degToRad(36)));

    p1[0] = Vector2dd(w / 2, h / 2);

    RGB24Buffer *buffer  = new RGB24Buffer(h, w, RGBColor::Black());
    AbstractPainter<RGB24Buffer> painter(buffer);

    PolygonCombiner combiner;
    combiner.pol[0] = p1;
    combiner.pol[1] = p2;

    combiner.prepare();
    Polygon p3 = combiner.intersection();

    combiner.drawDebug(buffer);
    painter.drawPolygon(p3, RGBColor::Magenta());


    BMPLoader().save("intersect.bmp", buffer);
    delete_safe(buffer);
}

int TEST_FILED_H = 400;
int TEST_FILED_W = 400;

Polygon testTwoPolygons(Polygon &p1, Polygon &p2, const std::string &name)
{

    RGB24Buffer *buffer  = new RGB24Buffer(TEST_FILED_H, TEST_FILED_W, RGBColor::Black());
    AbstractPainter<RGB24Buffer> painter(buffer);

    PolygonCombiner combiner;
    combiner.pol[0] = p1;
    combiner.pol[1] = p2;

    combiner.prepare();
    cout << "Structure after preparation" << endl;
    cout << combiner << endl;
    combiner.validateState();

    Polygon p3 = combiner.intersection();
    cout << "Structure after intersection" << endl;
    cout << combiner << endl;

    painter.drawPolygon(p1, RGBColor::Yellow());
    painter.drawPolygon(p2, RGBColor::Cyan());


    for (int i = 0; i < TEST_FILED_H; i++)
        for (int j = 0; j < TEST_FILED_W; j++)
        {
            if (p3.isInside(Vector2dd(j,i)))
            {
                buffer->element(i,j) = RGBColor::Gray();
            }
        }

    //painter.drawPolygon(p3, RGBColor::Magenta());
    /*for (size_t i = 0; i < p3.size(); i++)
    {
        Vector2dd point = p3.getPoint(i);
        buffer->drawLine(point, p3.getNextPoint(i), RGBColor::rainbow((double)i / (p3.size())));
        painter.drawFormat(point.x(), point.y(), RGBColor::White(), 1, "%d", i);
    }*/


    combiner.drawDebug(buffer);
    cout << "Result polygon: " << p3 << endl;
    cout << combiner << endl;

    BMPLoader().save(name, buffer);
    delete_safe(buffer);

    return p3;
}

TEST(polygon, twoSeparateIntersections)
{
    Polygon  p2 = Polygon::Reverse(Polygon::RegularPolygon(5, Vector2dd(TEST_FILED_W / 2 , TEST_FILED_H / 2), 180, degToRad(72)));
    p2[0] = Vector2dd(TEST_FILED_W / 2, TEST_FILED_H / 2);
    Polygon  p1 = p2;
    p2.transform(Matrix33::ShiftProj(TEST_FILED_W, 10) * Matrix33::MirrorYZ());
    testTwoPolygons(p1, p2, "poly-nonconvex-2.bmp");
}

TEST(polygon, trivialIntersection)
{
    Polygon  p2 = Polygon::Reverse(Polygon::RegularPolygon(5, Vector2dd(TEST_FILED_W / 2 , TEST_FILED_H / 2), 180, degToRad(72)));
    Polygon  p1 = p2.transformed(Matrix33::RotationZ(degToRad(36)));
    p2.transform(Matrix33::ShiftProj(TEST_FILED_W, 10) * Matrix33::MirrorYZ());
    testTwoPolygons(p1, p2, "poly-trivial.bmp");
}

TEST(polygon, collinearIntersection)
{
    Polygon  p2 = Polygon::FromRectagle(Rectangled::FromCorners(Vector2dd(60, 60), Vector2dd(TEST_FILED_W - 60,TEST_FILED_H - 60)));
    Polygon  p1 = Polygon::FromRectagle(Rectangled::FromCorners(Vector2dd(60,100), Vector2dd(TEST_FILED_W - 100,TEST_FILED_H - 60)));
    testTwoPolygons(p1, p2, "poly-collinear.bmp");
}

TEST(polygon, noIntersection)
{
    Polygon  p2 = Polygon::FromRectagle(Rectangled(Vector2dd(10,10), Vector2dd( 40, 40)));
    Polygon  p1 = Polygon::FromRectagle(Rectangled(Vector2dd(60,60), Vector2dd(290,290)));
    Polygon  p3 = testTwoPolygons(p1, p2, "poly-no-intersection.bmp");
    CORE_ASSERT_TRUE_P(p3.size() == 0, ("There should be no intersection. But we have polygon of size %d\n", (int)p3.size()));
}

TEST(polygon, intersectionTouch)
{
    Polygon  p2 = Polygon::FromRectagle(Rectangled(Vector2dd( 10, 10), Vector2dd( 100, 100)));
    Polygon  p1 = Polygon::FromRectagle(Rectangled(Vector2dd(110,110), Vector2dd(290,290)));
    Polygon  p3 = testTwoPolygons(p1, p2, "poly-touch.bmp");
    CORE_ASSERT_TRUE_P(p3.size() == 0, ("There should be no intersection. But we have polygon of size %d\n", (int)p3.size()));
}

TEST(polygon, insideIntersection)
{
    Polygon  p2 = Polygon::FromRectagle(Rectangled(Vector2dd(10,10), Vector2dd(250,250)));
    Polygon  p1 = Polygon::FromRectagle(Rectangled(Vector2dd(60,60), Vector2dd(140, 140)));
    Polygon  p3 = testTwoPolygons(p1, p2, "poly-inside.bmp");

    CORE_ASSERT_TRUE(p3[0].notTooFar(Vector2dd (60,  60)), "Fail with one poligon inside the other");
    CORE_ASSERT_TRUE(p3[1].notTooFar(Vector2dd(200,  60)), "Fail with one poligon inside the other");
    CORE_ASSERT_TRUE(p3[2].notTooFar(Vector2dd(200, 200)), "Fail with one poligon inside the other");
    CORE_ASSERT_TRUE(p3[3].notTooFar(Vector2dd( 60, 200)), "Fail with one poligon inside the other");
}


TEST(polygon, CameraView)
{
    CameraModel cam1, cam2;

    cam1.intrinsics = PinholeCameraIntrinsics(Vector2dd(400,400), degToRad(50));
    cam2.intrinsics = PinholeCameraIntrinsics(Vector2dd(400,400), degToRad(50));

    cam1.setLocation(Affine3DQ::Shift(-10, 0, 0) * Affine3DQ::RotationY(degToRad(10)));
    cam1.setLocation(Affine3DQ::Shift( 10, 0, 0) * Affine3DQ::RotationY(degToRad(-10)));


    Sphere3d s(Vector3dd::Zero(), 200);
    RaytraceableSphere sphere(s);
    Matrix44 sphereRot = Matrix44::RotationX(degToRad(90));

    Polygon in;
    in.push_back(Vector2dd(-200,-200));
//    in.push_back(Vector2dd(-100,-200));
    in.push_back(Vector2dd(-  0,-200));
//    in.push_back(Vector2dd( 100,-200));
    in.push_back(Vector2dd( 200,-200));

//    in.push_back(Vector2dd( 200,-100));
    in.push_back(Vector2dd( 200,-  0));
//    in.push_back(Vector2dd( 200, 100));
    in.push_back(Vector2dd( 200, 200));

//    in.push_back(Vector2dd( 100, 200));
    in.push_back(Vector2dd(   0, 200));
//    in.push_back(Vector2dd(-100, 200));
    in.push_back(Vector2dd(-200, 200));

//    in.push_back(Vector2dd(-200, 100));
    in.push_back(Vector2dd(-200,   0));
//    in.push_back(Vector2dd(-200,-100));
    in.push_back(Vector2dd(-200,-200));


    Mesh3D mesh;
    mesh.switchColor(true);
    CalibrationHelpers drawer;

    mesh.setColor(RGBColor::Red());
    drawer.drawCamera(mesh, cam1, 3.0);

    mesh.setColor(RGBColor::Green());
    drawer.drawCamera(mesh, cam2, 3.0);

    mesh.setColor(RGBColor::Blue());
    mesh.addIcoSphere(s, 2.0);

    std::vector<Vector3dd> proj1;
    std::vector<Vector3dd> proj2;

    Polygon p1, p2;

    for (int i = 0; i < in.size(); i++)
    {
        Ray3d ray1 = cam1.rayFromPixel(in[i]).normalised();
        RayIntersection r1;
        r1.ray = ray1;
        sphere.intersect(r1);
        proj1.push_back(r1.getPoint());

        Ray3d ray2 = cam2.rayFromPixel(in[i]).normalised();
        RayIntersection r2;
        r2.ray = ray2;
        sphere.intersect(r2);
        proj2.push_back(r2.getPoint());



        p1.push_back(Vector3dd::toSpherical(sphereRot * r1.getPoint()).xy());
        p2.push_back(Vector3dd::toSpherical(sphereRot * r2.getPoint()).xy());
    }

    p1.transform(Matrix33::ShiftProj(2000, 2000) * Matrix33::Scale2(700));
    p2.transform(Matrix33::ShiftProj(2000, 2000) * Matrix33::Scale2(700));


    PolygonCombiner combiner(p1,p2);
    combiner.prepare();
    Polygon p3 = combiner.intersection();

    int h = 4000;
    int w = 4000;
    RGB24Buffer *buffer  = new RGB24Buffer(h, w, RGBColor::Black());
    AbstractPainter<RGB24Buffer> painter(buffer);
    painter.drawPolygon(p1, RGBColor::Red());
    painter.drawPolygon(p2, RGBColor::Green());
    painter.drawPolygon(p3, RGBColor::Yellow());

    combiner.drawDebug(buffer);
    BMPLoader().save("spherical.bmp", buffer);
    delete_safe(buffer);

    mesh.setColor(RGBColor::Green());
    for (size_t i = 0; i < in.size(); i++)
    {
        mesh.addLine(proj1[i], proj1[(i + 1) % in.size()]);
        mesh.addLine(proj2[i], proj2[(i + 1) % in.size()]);
    }

    mesh.setColor(RGBColor::Yellow());
    for (size_t i = 0; i < p3.size(); i++)
    {
        Vector2dd curr = p3.getPoint(i);
        Vector2dd next = p3.getNextPoint(i);

        Vector3dd sph1 = sphereRot.inverted() * Vector3dd::FromSpherical(curr.x(), curr.y(), s.r);
        Vector3dd sph2 = sphereRot.inverted() * Vector3dd::FromSpherical(next.x(), next.y(), s.r);

        mesh.addLine(sph1, sph2);
    }

    mesh.dumpPLY("out.ply");

}
