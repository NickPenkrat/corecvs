#ifndef DEFORMMAP_H
#define DEFORMMAP_H

#include "vector2d.h"

namespace corecvs {

/**
 * This class holds the mapping function from coordinate domain
 * to the other coordinate domain.
 *
 * We probably should use only static poly-morphism.
 * So map should be made non-virtual
 **/
template<typename InputIndexType, typename OutputIndexType>
class DeformMap
{
public:

    typedef Vector2d<InputIndexType>   InputPoint;
    typedef Vector2d<OutputIndexType> OutputPoint;

    inline OutputPoint map(const InputIndexType & /*y*/, const InputIndexType & /*x*/) {}

    //inline OutputPoint map(const InputPoint &point)
    //{
    //    return map (point.y, point.x);
    //}

    void getCircumscribedImageRect(const InputIndexType &x1, const InputIndexType &y1, const InputIndexType &x2, const InputIndexType &y2,
                                   OutputPoint &min, OutputPoint &max)
    {
        min = map(y1,x1);
        max = map(y2,x2);

        for (int i = y1; i <= y2; i++)
        {
            OutputPoint mapLeft  = map(i, x1);
            OutputPoint mapRight = map(i, x2);
            if (mapLeft .x() < min.x()) min.x() = mapLeft.x();
            if (mapRight.x() > max.x()) max.x() = mapRight.x();
        }

        for (int j = x1; j <= x2; j++)
        {

            OutputPoint mapTop    = map(y1, j);
            OutputPoint mapBottom = map(y2, j);
            if (mapTop   .y() < min.y()) min.y() = mapTop.y();
            if (mapBottom.y() > max.y()) max.y() = mapBottom.y();
        }
    }

    void getInscribedImageRect(const InputIndexType &x1, const InputIndexType &y1, const InputIndexType &x2, const InputIndexType &y2,
                               OutputPoint &min, OutputPoint &max)
    {
        getCircumscribedImageRect(x2, y2, x1, x2, min, max);
    }


    virtual ~DeformMap() {}
};

} // namespace corecvs


#endif // DEFORMMAP_H
