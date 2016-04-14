/**
 * \file main_test_draw.cpp
 * \brief This is the main file for the test draw
 *
 * \date Apr 19, 2011
 * \author alexander
 *
 * \ingroup autotest
 */

#include <iostream>
#include "gtest/gtest.h"

#include "global.h"

#include "rgb24Buffer.h"
#include "bmpLoader.h"
#include "abstractPainter.h"
#include "simpleRenderer.h"
#include "mesh3d.h"
#include "calibrationCamera.h"


using namespace std;
using namespace corecvs;

TEST(Draw, testCircles)
{
    RGB24Buffer *buffer = new RGB24Buffer(21, 52);

    RGBColor colors[] = {
        RGBColor::Red(),
        RGBColor::Blue(),
        RGBColor::Yellow(),
        RGBColor::Black(),
        RGBColor::Indigo()
    };

    for (int i = 11; i >=1; i-= 1)
    {
        AbstractPainter<RGB24Buffer>(buffer).drawCircle(10,10, i, colors[i % CORE_COUNT_OF(colors)] );
    }

    for (int i = 11; i >=1; i-= 2)
    {
        buffer->drawArc(33, 10, i, colors[i % CORE_COUNT_OF(colors)] );
    }

    BMPLoader().save("circles.bmp", buffer);
    delete_safe(buffer);
}

TEST(Draw, testCircles1)
{
    RGB24Buffer *buffer = new RGB24Buffer(100, 100);

    buffer->drawArc(50, 50, 40, RGBColor::White() );

    for (int i = 0; i < 40; i++ )
    {
        Vector2dd point = Vector2dd(50.0, 50.0) + Vector2dd::FromPolar(degToRad(360.0 / 40 * i), 40);
        buffer->setElement(fround(point.y()), fround(point.x()), RGBColor::Blue());
    }

    BMPLoader().save("circles1.bmp", buffer);
    delete_safe(buffer);
}

TEST(Draw, testCircleIterator)
{
    RGB24Buffer *buffer = new RGB24Buffer(100, 100);


    CircleSpanIterator inner(Circle2d(50, 50, 30));
    while (inner.step())
    {
        LineSpanInt span = inner.getSpan();
        do {
            if (buffer->isValidCoord(span.pos())) {
                buffer->element(span.pos()) = RGBColor::Red();
            }
        } while (span.step());
    }


    BMPLoader().save("circles-it.bmp", buffer);
    delete_safe(buffer);
}

TEST(Draw, testRectangles)
{
    int h = 100;
    int w = 100;

    RGB24Buffer *buffer = new RGB24Buffer(h, w, RGBColor::Black());

    for (int i = 1; i < 8; i++) {
        int pos = (i* (i - 1)) / 2 + 1;

        buffer->drawRectangle( pos,  pos, i, i, RGBColor::Red()  , 0);

        buffer->drawRectangle(50 + pos,      pos, i, i, RGBColor::Green(), 1);
        buffer->drawRectangle(     pos, 50 + pos, i, i, RGBColor::Blue() , 2);
    }

    BMPLoader().save("rects.bmp", buffer);
    delete_safe(buffer);
}




TEST(Draw, testSpanDraw)
{
    int h = 200;
    int w = 200;

    RGB24Buffer *buffer = new RGB24Buffer(h, w, RGBColor::Black());
    {
        TrapezoidSpanIterator it(10, 40, 10, 30, 40, 90);
        while (it.step())
        {
            LineSpanInt span = it.getSpan();
            buffer->drawHLine(span.x1, span.y(), span.x2, RGBColor::Red());
        }
    }

    {
        TrapezoidSpanIterator it(110, 170, 10, 30, 5, 35);
        while (it.step())
        {
            LineSpanInt span = it.getSpan();
            buffer->drawHLine(span.x1, span.y(), span.x2, RGBColor::Green());
        }
    }

    {
        Triangle2dd t(Vector2dd(120, 30), Vector2dd(170, 80), Vector2dd(140, 10));
        TriangleSpanIterator it(t);
        while (it.step())
        {
            LineSpanInt span = it.getSpan();
            buffer->drawHLine(span.x1, span.y(), span.x2, RGBColor::Pink());
        }
    }

    BMPLoader().save("spandraw.bmp", buffer);
    delete_safe(buffer);
}

TEST(Draw, testSpanDrawTriangle)
{
    int h = 200;
    int w = 200;

    RGB24Buffer *buffer = new RGB24Buffer(h, w, RGBColor::Black());

    Triangle2dd t[] = {
        Triangle2dd(Vector2dd(10, 10), Vector2dd( 70, 10), Vector2dd(10, 90)),
        Triangle2dd(Vector2dd(70, 90), Vector2dd( 70, 10), Vector2dd(10, 90)),

        Triangle2dd(Vector2dd(110, 10), Vector2dd( 170, 10), Vector2dd(110, 90)),
        Triangle2dd(Vector2dd(171, 90), Vector2dd( 171, 10), Vector2dd(111, 90))
    };

    RGBColor c[] = {RGBColor::Pink(), RGBColor::Cyan(), RGBColor::Pink(), RGBColor::Cyan()};

    for (size_t i = 0; i < CORE_COUNT_OF(t); i++) {

        TriangleSpanIterator it(t[i]);
        while (it.step())
        {
            LineSpanInt span = it.getSpan();
            buffer->drawHLine(span.x1, span.y(), span.x2, c[i]);
        }
    }

    BMPLoader().save("triangledraw.bmp", buffer);
    delete_safe(buffer);

}

TEST(Draw, testAttributedTriangle)
{
    int h = 300;
    int w = 300;
    RGB24Buffer *buffer = new RGB24Buffer(h, w, RGBColor::Black());

    AttributedTriangle t(Vector2dd(0,0), Vector2dd(0,300), Vector2dd(300,150));
    RGBColor c[3] = {RGBColor::Red(), RGBColor::Green(), RGBColor::Blue()};

    for (int i = 0; i < 3; i++) {
        t.p[i].attributes.push_back(c[i].r());
        t.p[i].attributes.push_back(c[i].g());
        t.p[i].attributes.push_back(c[i].b());
    }

    AttributedTriangleSpanIterator it(t);

    while (it.step())
    {
        AttributedLineSpan span = it.getAttrSpan();
        do {
            if (buffer->isValidCoord(span.pos()) ) {
                Vector3dd color(span.att()[0], span.att()[1], span.att()[2]);
                buffer->element(span.pos()) = RGBColor::FromDouble(color);
            }
        } while (span.step());
    }

    BMPLoader().save("attributed.bmp", buffer);
}

TEST(Draw, testSpanRenderTriangle)
{
    int h = 300;
    int w = 300;

    RGB24Buffer *buffer = new RGB24Buffer(h, w, RGBColor::Black());

    Triangle3dd t[2] = {
        Triangle3dd(Vector3dd(-100,-100, 100), Vector3dd(-100, 100, 100), Vector3dd( 100, 0, 120)),
        Triangle3dd(Vector3dd( 100,-100, 100), Vector3dd( 100, 100, 100), Vector3dd(-100, 0, 120)),
    };

    Triangle2dd p[CORE_COUNT_OF(t)];
    for (size_t i = 0; i < CORE_COUNT_OF(t); i++)
    {
        p[i] = Triangle2dd(t[i].p1().xy() + Vector2dd(100.0, 100.0),
                           t[i].p2().xy() + Vector2dd(100.0, 100.0),
                           t[i].p3().xy() + Vector2dd(100.0, 100.0));
    }



    RGBColor c[CORE_COUNT_OF(t)] = {RGBColor::Pink(), RGBColor::Cyan()};

    for (size_t i = 0; i < CORE_COUNT_OF(t); i++) {

        TriangleSpanIterator it(p[i]);
        while (it.step())
        {
            LineSpanInt span = it.getSpan();
            buffer->drawHLine(span.x1, span.y(), span.x2, c[i]);
        }
    }
    BMPLoader().save("renderdraw.bmp", buffer);


    /* Z buffer */
    AbstractBuffer<double> *zBuffer = new AbstractBuffer<double>(h,w, /*std::numeric_limits<double>::max()*/ 140.0);

    AttributedTriangle a[CORE_COUNT_OF(t)];
    for (size_t i = 0; i < CORE_COUNT_OF(t); i++)
    {
        a[i] = AttributedTriangle(t[i].p1().xy() + Vector2dd(100.0, 100.0),
                                  t[i].p2().xy() + Vector2dd(100.0, 100.0),
                                  t[i].p3().xy() + Vector2dd(100.0, 100.0));
        a[i].p1().attributes.push_back(t[i].p1().z());
        a[i].p2().attributes.push_back(t[i].p2().z());
        a[i].p3().attributes.push_back(t[i].p3().z());
    }


    for (size_t i = 0; i < CORE_COUNT_OF(t); i++) {

        AttributedTriangleSpanIterator it(a[i]);

        while (it.step())
        {
            LineSpanInt span = it.getSpan();
            double z1 = it.part.a1[0];
            double z2 = it.part.a2[0];

            for (int j = span.x1; j <= span.x2; j++) {
                if (zBuffer->isValidCoord(span.y(), j) ) {
                    double z = lerp(z1,z2, j, span.x1, span.x2 + 1);
//                    SYNC_PRINT(("Z=%lf, WAS=%lf\n", z, zBuffer->element(span.y(), j)));

                    if (zBuffer->element(span.y(), j) >  z) {
                         zBuffer->element(span.y(), j) = z;
                         buffer->element(span.y(), j) = c[i];
                    }
                }
            }
        }
    }
    BMPLoader().save("renderdraw-rgb.bmp", buffer);





    buffer->drawDoubleBuffer(zBuffer);
    BMPLoader().save("renderdraw-z.bmp", buffer);
    delete_safe(buffer);
    delete_safe(zBuffer);

}


TEST(Draw, testFloodFill)
{
    int h = 20;
    int w = 20;

    RGB24Buffer *buffer = new RGB24Buffer(h, w, RGBColor::Black());
    AbstractPainter<RGB24Buffer> painter(buffer);
    painter.drawCircle(    w / 4,     h / 4, w / 5, RGBColor::Red() );
    painter.drawCircle(    w / 4, 3 * h / 4, w / 5, RGBColor::Red() );
    painter.drawCircle(3 * w / 4,     h / 4, w / 5, RGBColor::Red() );
    painter.drawCircle(3 * w / 4, 3 * h / 4, w / 5, RGBColor::Red() );

    BMPLoader().save("flood_before.bmp", buffer);

    AbstractPainter<RGB24Buffer>::EqualPredicate predicate(RGBColor::Black(), RGBColor::Blue());
    painter.floodFill(w / 2, h / 2, predicate);

    BMPLoader().save("flood_after.bmp", buffer);
    //printf("Predicate  : %d\n", predicate.countPred);
    //printf("Mark       : %d\n", predicate.countMark);
    //printf("Double Mark: %d\n", predicate.doubleMark);

    delete_safe(buffer);
}

TEST(Draw, testRobot)
{
    int h = 300;
    int w = 300;

    RGB24Buffer *buffer = new RGB24Buffer(h, w, RGBColor::Black());
    AbstractPainter<RGB24Buffer> painter(buffer);

    double r = 20;
    Circle2d center  (150,150, r);
    Circle2d perifery[4] =
    {
        Circle2d(100, 110, r),
        Circle2d(200, 110, r),

        Circle2d(110, 190, r),
        Circle2d(190, 190, r)
    };

    double l1 = 50*1.4 - r;
    double l2 = 50*1.4 + r ;



    painter.drawCircle(center, RGBColor::Blue());

    painter.drawCircle( perifery[0], RGBColor::Yellow());
    painter.drawCircle( perifery[2], RGBColor::Yellow());

    painter.drawCircle( perifery[1], RGBColor::Red());
    painter.drawCircle( perifery[3], RGBColor::Red());

    BMPLoader().save("robot.bmp", buffer);

    AbstractBuffer<double> acc(buffer->getSize());

   /** */

    CircleSpanIterator outer(center);
    while (outer.step())
    {
        LineSpanInt span = outer.getSpan();
        for (int s = span.x1; s < span.x2; s++ )
        {
            CircleSpanIterator inner(Circle2d(s, span.y(), r/2));
            while (inner.step())
            {
                LineSpanInt span = inner.getSpan();
                for (int s1 = span.x1; s1 < span.x2; s1++ )
                {
                    if (acc.isValidCoord(span.y(), s1))
                    {
                        acc.element(span.y(), s1)++;
                    }
                }
            }
        }
    }

    /**/
    for (int c = 0; c < 4; c++)
    {
        CircleSpanIterator outer(perifery[c]);
        while (outer.step())
        {
            LineSpanInt span = outer.getSpan();
            for (int s = span.x1; s < span.x2; s++ )
            {
                CircleSpanIterator inner(Circle2d(s, span.y(), l2));
                while (inner.step())
                {
                    LineSpanInt span = inner.getSpan();
                    for (int s1 = span.x1; s1 < span.x2; s1++ )
                    {
                        //SYNC_PRINT(("#"));
                        if (acc.isValidCoord(span.y(), s1))
                        {
                            acc.element(span.y(), s1)++;
                        }
                    }
                }

                CircleSpanIterator inner1(Circle2d(s, span.y(), l1));
                while (inner1.step())
                {
                    LineSpanInt span = inner1.getSpan();
                    for (int s1 = span.x1; s1 < span.x2; s1++ )
                    {
                        //SYNC_PRINT(("#"));
                        if (acc.isValidCoord(span.y(), s1))
                        {
                            acc.element(span.y(), s1)--;
                        }
                    }
                }
            }

        }

    }


    buffer->drawDoubleBuffer(acc);
    BMPLoader().save("robot2.bmp", buffer);

    delete_safe(buffer);
}
