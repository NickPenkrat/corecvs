#ifndef SIMPLERENDERER_H
#define SIMPLERENDERER_H

#include "mathUtils.h"
#include "matrix44.h"
#include "polygons.h"


namespace corecvs {

class LineSpanInt {
public:
    int y;
    int x1;
    int x2;
};

/**
 * trapezoid 2d iterator
 **/
class TrapezoidSpanIterator
{
public:
    int y1;
    int y2;

    int x11;
    int x12;

    int x21;
    int x22;

    int currentY;
    double x1, x2;
    double dx1;
    double dx2;

    TrapezoidSpanIterator(int y1, int y2, int x11, int x12, int x21, int x22) :
        y1(y1) , y2(y2), x11(x11), x12(x12), x21(x21), x22(x22)
    {
        currentY = y1;
        double dy = y2 - y1;
        dx1 = (x12 - x11) / dy;
        dx2 = (x22 - x21) / dy;
        x1 = x11 - dx1;
        x2 = x12 - dx2;
    }

    bool step()
    {
        currentY++;
        x1 += dx1;
        x2 += dx2;
        return (currentY <= y2);
    }

    void getSpan(int &y, int &x1, int &x2)
    {
        y = currentY;
        x1 = fround(this->x1);
        x2 = fround(this->x2);
    }

    LineSpanInt getSpan()
    {
        LineSpanInt span;
        getSpan(span.y, span.x1, span.x2);
        return span;
    }
};

/*
class TriangleSpanIterator : TrapezoidSpanIterator
{
public:
    TriangleSpanIterator(const Triangle2dd &triangle) :

    {
    }
};
*/

class SimpleRenderer
{
public:
    SimpleRenderer();


    Matrix44 modelviewMatrix;

};

} // namespace corecvs

#endif // SIMPLERENDERER_H
