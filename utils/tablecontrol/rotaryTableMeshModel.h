#ifndef ROTARYTABLEMESHMODEL_H
#define ROTARYTABLEMESHMODEL_H

#include "mesh3d.h"
#include "calibrationCamera.h"

using corecvs::Mesh3D;

class RotaryTableMeshModel
{
public:
    RotaryTableMeshModel();

    static Mesh3D getMesh(const CameraLocationAngles &state);
};

#endif // ROTARYTABLEMESHMODEL_H
