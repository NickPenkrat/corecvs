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
            buffer->drawHLine(span.x1, span.y, span.x2, RGBColor::Red());
        }
    }

    {
        TrapezoidSpanIterator it(110, 170, 10, 30, 5, 35);
        while (it.step())
        {
            LineSpanInt span = it.getSpan();
            buffer->drawHLine(span.x1, span.y, span.x2, RGBColor::Green());
        }
    }

    {
        Triangle2dd t(Vector2dd(120, 30), Vector2dd(170, 80), Vector2dd(140, 10));
        TriangleSpanIterator it(t);
        while (it.step())
        {
            LineSpanInt span = it.getSpan();
            buffer->drawHLine(span.x1, span.y, span.x2, RGBColor::Pink());
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
            buffer->drawHLine(span.x1, span.y, span.x2, c[i]);
        }
    }

    BMPLoader().save("triangledraw.bmp", buffer);
    delete_safe(buffer);

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
