#ifndef LINE_SPAN_H
#define LINE_SPAN_H

#include <algorithm>
#include <vector>

#include <vector2d.h>

namespace corecvs {


class LineSpanInt {
public:
    int cy;
    int x1;
    int x2;

    LineSpanInt() {}

    LineSpanInt(int y, int x1, int x2) :
        cy(y), x1(x1), x2(x2)
    {}

    void clip(int w, int h) {
        if (x1 > x2) std::swap(x1, x2);
        if (x1 <  0) x1 = 0;
        if (x2 >= w) x2 = w - 1;

        if (x1 >= w || x2 < 0 || cy < 0 || cy >= h ) {
            x1 = 1;
            x2 = 0;
        }
    }

    bool step() {
        x1++;
        return (x1 <= x2);
    }

    int x() {
        return x1;
    }

    int y() {
        return cy;
    }

    Vector2d<int> pos() {
        return Vector2d<int>(x(), y());
    }


};

typedef std::vector<double> FragmentAttributes;

class AttributedLineSpan : public LineSpanInt {
public:
    FragmentAttributes att1;
    FragmentAttributes att2;
    FragmentAttributes catt;
    FragmentAttributes datt;


    bool step() {
        for (size_t i = 0; i < att1.size(); i++) {
            catt[i] += datt[i];
        }
        return LineSpanInt::step();
    }

    FragmentAttributes att() {
        return catt;
    }

};

} // namespace corecvs

#endif // LINE_SPAN_H
