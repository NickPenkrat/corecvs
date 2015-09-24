#ifndef CALIBRATIONHELPERS_H
#define CALIBRATIONHELPERS_H

#include "mesh3d.h"
#include "calibrationPhotostation.h"

namespace corecvs {

class CalibrationHelpers
{
public:
    static RGBColor palette[];


    static void drawPly(Mesh3D &mesh, Photostation &ps, double scale = 50.0);
    static void drawPly(Mesh3D &mesh, ObservationList &list);
    static void drawPly(Mesh3D &mesh, Photostation &ps, ObservationList &list, double scale);
};

} // namespace corecvs

#endif // CALIBRATIONHELPERS_H
