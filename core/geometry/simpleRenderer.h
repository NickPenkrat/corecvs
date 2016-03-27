#ifndef SIMPLERENDERER_H
#define SIMPLERENDERER_H

#include "matrix44.h"


namespace corecvs {

class LineSpanInt {
public:
    int y;
    int x1;
    int x2;
};


class SimpleRenderer
{
public:
    SimpleRenderer();


    Matrix44 modelviewMatrix;

};

} // namespace corecvs

#endif // SIMPLERENDERER_H
