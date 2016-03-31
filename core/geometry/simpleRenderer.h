#ifndef SIMPLERENDERER_H
#define SIMPLERENDERER_H

/**
 *  \file  simpleRenderer.h
 **/

#include "mathUtils.h"
#include "matrix44.h"
#include "polygons.h"
#include "abstractPainter.h"


namespace corecvs {

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

    TrapezoidSpanIterator() {}

    TrapezoidSpanIterator(int y1, int y2, int x11, int x12, int x21, int x22) :
        y1(y1) , y2(y2), x11(x11), x12(x12), x21(x21), x22(x22)
    {
        SYNC_PRINT(("TrapezoidSpanIterator::TrapezoidSpanIterator(%d %d %d %d %d %d): called\n", y1, y2, x11, x12, x21, x22 ));
        currentY = y1;
        double dy = y2 - y1;
        dx1 = (x21 - x11) / dy;
        dx2 = (x22 - x12) / dy;
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

class TriangleSpanIterator
{
public:
    Triangle2dd sortedt;
    TrapezoidSpanIterator part;


    TriangleSpanIterator(const Triangle2dd &triangle)
    {
        sortedt = triangle;
        if (sortedt.p1.y() > sortedt.p2.y()) std::swap(sortedt.p1, sortedt.p2);
        if (sortedt.p2.y() > sortedt.p3.y()) std::swap(sortedt.p2, sortedt.p3);
        if (sortedt.p1.y() > sortedt.p2.y()) std::swap(sortedt.p1, sortedt.p2);

/*        cout << "Sorted by Y" << endl;
        cout << "  " << sortedt.p1 << endl;
        cout << "  " << sortedt.p2 << endl;
        cout << "  " << sortedt.p3 << endl;*/

        double longslope = (sortedt.p3.x() - sortedt.p1.x()) / (sortedt.p3.y() - sortedt.p1.y());
        double centerx1 = sortedt.p1.x() + longslope * (sortedt.p2.y() - sortedt.p1.y());
        double centerx2 = sortedt.p2.x();

        if (centerx1 > centerx2)
            std::swap(centerx1, centerx2);
        part = TrapezoidSpanIterator(sortedt.p1.y(), sortedt.p2.y(), sortedt.p1.x(), sortedt.p1.x(), centerx1, centerx2);
    }

    bool step()
    {
        if (!part.step())
        {
//            SYNC_PRINT(("Changing iterator: %d %d\n", part.currentY >= sortedt.p3.y()));
            if (part.currentY >= sortedt.p3.y()) {
                return false;
            }
            part = TrapezoidSpanIterator(sortedt.p2.y(), sortedt.p3.y(), part.x21, part.x22, sortedt.p3.x(), sortedt.p3.x());
            return part.step();
        }
        return true;
    }

    void getSpan(int &y, int &x1, int &x2)
    {
        part.getSpan(y, x1, x2);
    }

    LineSpanInt getSpan()
    {
        return part.getSpan();
    }
};

class Mesh3D;
class RGB24Buffer;


class SimpleRenderer
{
public:
    SimpleRenderer();
    Matrix44 modelviewMatrix;

    void render (Mesh3D *mesh, RGB24Buffer *buffer);


};

} // namespace corecvs

#endif // SIMPLERENDERER_H
