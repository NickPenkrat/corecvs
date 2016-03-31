#ifndef LINE_SPAN_H
#define LINE_SPAN_H

#include <algorithm>

namespace corecvs {


class LineSpanInt {
public:
    int y;
    int x1;
    int x2;

    LineSpanInt() {}

    LineSpanInt(int y, int x1, int x2) :
        y(y), x1(x1), x2(x2)
    {}

    void clip(int w, int h) {
        if (x1 > x2) std::swap(x1, x2);
        if (x1 <  0) x1 = 0;
        if (x2 >= w) x2 = w - 1;

        if (x1 >= w || x2 < 0 || y < 0 || y >= h ) {
            x1 = 1;
            x2 = 0;
        }
    }

};

} // namespace corecvs

#endif // LINE_SPAN_H
