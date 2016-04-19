#ifndef POLYGONPOINTITERATOR_H
#define POLYGONPOINTITERATOR_H

#include "polygons.h"
#include "simpleRenderer.h"

namespace corecvs {

/**
 * NOT YET WORKING!!!!
 * So far we only support convex polygon
 **/
class PolygonSpanIterator
{
public:
    const Polygon &polygon;

    TrapezoidSpanIterator part;

    int indexdelta;

    int cand1;
    int cand2;

    int deep  ;
    int shallow;


    vector<int>  sortedIndex;
    vector<bool> side;
    unsigned currentIndex;

    Vector2dd getPoint(int i) {
        return polygon[sortedIndex[i]];
    }

    PolygonSpanIterator(const Polygon &polygon) : polygon(polygon)
    {
        side.resize(polygon.size());
        sortedIndex.reserve(polygon.size());
        for (unsigned i = 0; i < polygon.size(); i++)
            sortedIndex.push_back(i);

        std::sort(sortedIndex.begin(), sortedIndex.end(), [=](int a, int b) { return polygon[a].y() < polygon[b].y(); });
        int idx = sortedIndex.front() + 1;
        while (idx != sortedIndex.back()) {
            side[idx] = true;
            idx = (idx + 1) % polygon.size();
        }

        currentIndex = sortedIndex[0];
        Vector2dd origin = polygon[currentIndex];

        cout << "Top Point:" << origin <<  endl;

        cout << "Bottom Point:" << polygon[sortedIndex.back()] << endl;

        cand1 = (currentIndex + polygon.size() - 1) % polygon.size(); /* Left  slope driver */
        cand2 = (currentIndex + 1) % polygon.size();                  /* Right slope driver */

        deep    = cand1;
        shallow = cand2;

        cout << "Indexes:" << currentIndex << " " << deep << " " << shallow << endl;

        if (polygon.y(deep) < polygon.y(shallow)) std::swap(deep, shallow);

        cout << "Y:" << polygon.y(deep) << " " << polygon.y(shallow) << endl;


        double longslope = (polygon.x(deep) - origin.x()) / (polygon.y(deep) - origin.y());
        double centerx1 = origin.x() + longslope * (polygon.y(shallow) - origin.y());
        double centerx2 = polygon.x(shallow);

        if (centerx2 < centerx1) std::swap(centerx2, centerx1);


        part = TrapezoidSpanIterator(origin.y(), polygon.y(shallow), origin.x(), origin.x(), centerx1, centerx2);

    }

    void step()
    {
        SYNC_PRINT(("PolygonSpanIterator::step(): called\n"));
        part.step();

        if (!part.hasValue() && hasValue())
        {
            int cand1n = cand1;
            int cand2n = cand2;

            bool side = 0;
            /* First side had finished */
            if (shallow == cand1n) {
                cand1n = (cand1n + polygon.size() - 1) % polygon.size();
                side = 1;
            }

            /* Second side had finished */
            if (shallow == cand2n) {
                cand2n = (cand2n + 1) % polygon.size();

            }
#if 0
            cand1 = cand1n;
            cand2 = cand2n;

            deep    = cand1n;
            shallow = cand2n;

            cout << "Indexes:" << currentIndex << " " << deep << " " << shallow << endl;
            Vector2dd leftEnd  =  ;
            Vector3dd rightEnd =  ;s

            if (polygon.y(deep) < polygon.y(shallow))
                std::swap(deep, shallow);

            cout << "Y:" << polygon.y(deep) << " " << polygon.y(shallow) << endl;


            double longslope = (polygon.x(deep) - part.x21) / (polygon.y(deep) - part.y2);
            double centerx1 = part.x21 + longslope * (polygon.y(shallow) - part.y2);
            double centerx2 = polygon.x(shallow);

            if (centerx2 < centerx1) std::swap(centerx2, centerx1);

            /**/
            part = TrapezoidSpanIterator(part.y2, polygon.y(shallow), part.x21, part.x22, centerx1, centerx2);

            if (part.hasValue()) {
                part.step();
            }
            //part.currentY++;
#endif
            return;
        }


    }

    void getSpan(int &y, int &x1, int &x2)
    {
        part.getSpan(y, x1, x2);
    }

    LineSpanInt getSpan()
    {
        LineSpanInt span;
        getSpan(span.cy, span.x1, span.x2);
        return span;
    }

    bool hasValue() {
        return (part.currentY <= polygon.y(sortedIndex.back()));
    }

    /**
     * C++ style iteration
     **/
    PolygonSpanIterator &begin() {
        return *this;
    }

    PolygonSpanIterator & end() {
        return *this;
    }

    bool operator !=(const PolygonSpanIterator & /*other*/) {
        return this->hasValue();
    }

    LineSpanInt operator *() {
        return getSpan();
    }

    void operator ++() {
        step();
    }

};

class PolygonPointIterator
{
public:
    const Polygon &polygon;
    size_t petle;
    Triangle2dd triangle;
    TrianglePointIterator it;


    Triangle2dd getTriangle(int petle) {
        return Triangle2dd(polygon[0], polygon[petle - 1], polygon[petle]);
    }

    PolygonPointIterator(const Polygon &polygon) :
        polygon(polygon),
        petle(2),
        triangle(getTriangle(petle)),
        it(triangle)
    {}

    void step() {
        it.step();
        if (!it.hasValue()) {
            petle++;
            if (hasValue()) {
                triangle = getTriangle(petle);
                it = TrianglePointIterator(triangle);
            }
        }
    }

    bool hasValue() {
        return petle < polygon.size();
    }

    Vector2d<int> pos() {
        return it.pos();
    }


    /**
     * C++ style iteration
     **/
    PolygonPointIterator &begin() {
        return *this;
    }

    PolygonPointIterator & end() {
        return *this;
    }

    bool operator !=(const PolygonPointIterator & /*other*/) {
        return this->hasValue();
    }

    Vector2d<int> operator *() {
        return pos();
    }

    void operator ++() {
        step();
    }

};

} // namespace corecvs

#endif // POLYGONPOINTITERATOR_H
