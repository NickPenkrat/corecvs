#ifndef ROTARYTABLEMESHMODEL_H
#define ROTARYTABLEMESHMODEL_H

#include "core/geometry/mesh3d.h"
#include "core/cameracalibration/calibrationCamera.h"

using corecvs::Mesh3D;

class RotaryTableMeshModel
{
public:
    RotaryTableMeshModel();

    static Mesh3D getMesh(const CameraLocationAngles &state);
};

#endif // ROTARYTABLEMESHMODEL_H
