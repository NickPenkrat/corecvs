#pragma once

#include "rgbColor.h"

namespace corecvs {

class Mesh3D;
class CameraModel;
class Photostation;
class ObservationList;
class CalibrationFeaturePoint;
class CalibrationScene;

class CalibrationHelpers
{
public:

    enum RenderStyle {
        COLOR_PER_CAM,
        COLOR_PER_STATION
    };

    static RGBColor palette[];

    bool    printNames;
    bool    privateColor;
    bool    largePoints;

    CalibrationHelpers()
      : printNames(false)
      , privateColor(false)
    {}

    void drawCamera (Mesh3D &mesh, CameraModel &cam, double scale);

    void drawPly    (Mesh3D &mesh, Photostation &ps, double scale = 50.0);
    void drawPly    (Mesh3D &mesh, ObservationList &list);
    void drawPly    (Mesh3D &mesh, Photostation &ps, ObservationList &list, double scale);
    void drawPly    (Mesh3D &mesh, CalibrationFeaturePoint &fp, double scale);

    void drawScene  (Mesh3D &mesh, CalibrationScene &scene, double scale = 1.0);
};

} // namespace corecvs
