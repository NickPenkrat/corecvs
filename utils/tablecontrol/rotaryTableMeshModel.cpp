#include "rotaryTableMeshModel.h"

RotaryTableMeshModel::RotaryTableMeshModel()
{
}

Mesh3D RotaryTableMeshModel::getMesh(const CameraLocationAngles &state)
{
    Mesh3D mesh;
    mesh.switchColor(true);
    int stackstate = mesh.transformStack.size();

    mesh.setColor(RGBColor::Red());
    mesh.addAOB(Vector3dd(-2,-2,-2), Vector3dd( 2, 2,0));
    mesh.mulTransform(Matrix33::RotationZ(state.yaw()));

    mesh.setColor(RGBColor::Green());
    mesh.addAOB(Vector3dd(-2,-2,0), Vector3dd( 2,  2,1));
    mesh.addAOB(Vector3dd(-2,-2,1), Vector3dd(-1.5,2,3));
    mesh.addAOB(Vector3dd(1.5,-2,1), Vector3dd(2,2,3));

    mesh.mulTransform(Matrix44::Shift(0,0,2));
    mesh.mulTransform(Matrix33::RotationX(state.pitch()));

    mesh.setColor(RGBColor::Blue());
    mesh.addAOB(Vector3dd(-1,-1,0), Vector3dd( 1, 1, 0.5));

    mesh.mulTransform(Matrix44::Shift(0,0,0.5));
    mesh.mulTransform(Matrix33::RotationZ(state.roll()));
    mesh.setColor(RGBColor::Yellow());
    mesh.addAOB(Vector3dd(-1,-1,0), Vector3dd( 1, 1, 0.5));

    while (mesh.transformStack.size() > stackstate) {
        mesh.popTransform();
    }

    return mesh;
}
