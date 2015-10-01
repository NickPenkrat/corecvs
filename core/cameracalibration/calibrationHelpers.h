#ifndef CALIBRATIONHELPERS_H
#define CALIBRATIONHELPERS_H

#include "mesh3d.h"
#include "calibrationPhotostation.h"
#include "calibrationScene.h"

namespace corecvs {

class CalibrationHelpers
{
public:

    enum RenderStyle {
        COLOR_PER_CAM,
        COLOR_PER_STATION
    };

    static RGBColor palette[];

    bool printNames;
    bool privateColor;
    bool largePoints;

    CalibrationHelpers() :
        printNames(false),
        privateColor(false)
    {}

    void drawPly(Mesh3D &mesh, Photostation &ps, double scale = 50.0);
    void drawPly(Mesh3D &mesh, ObservationList &list);
    void drawPly(Mesh3D &mesh, Photostation &ps, ObservationList &list, double scale);

    void drawPly(Mesh3D &mesh, CalibrationFeaturePoint &fp, double scale);


    void drawScene(Mesh3D &mesh, CalibrationScene &scene, double scale = 1.0);
};

} // namespace corecvs

#endif // CALIBRATIONHELPERS_H
