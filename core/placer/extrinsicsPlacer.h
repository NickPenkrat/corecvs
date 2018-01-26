#ifndef EXTRINSICSPLACER_H
#define EXTRINSICSPLACER_H

#include "core/cameracalibration/calibrationLocation.h"

namespace corecvs {

class SimplifiedScene
{
public:
    std::vector<CameraLocationData> cameras;
    std::vector<Vector3dd> points;
    std::vector<std::vector<Vector3dd>> obs;


    double cost()
    {
        return 0;
    }

};

/** Most trival placer that optimises rays from observations by moving the cameras **/
class ExtrinsicsPlacer
{
public:
    ExtrinsicsPlacer();
};


}

#endif // EXTRINSICSPLACER_H
