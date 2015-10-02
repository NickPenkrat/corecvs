#pragma once

#include "rgbColor.h"

namespace corecvs {

class Mesh3D;
class Photostation;
class ObservationList;

class CalibrationHelpers
{
public:
    static RGBColor palette[];


    static void drawPly(Mesh3D &mesh, Photostation &ps, double scale = 50.0);
    static void drawPly(Mesh3D &mesh, ObservationList &list);
    static void drawPly(Mesh3D &mesh, Photostation &ps, ObservationList &list, double scale);
};

} // namespace corecvs
