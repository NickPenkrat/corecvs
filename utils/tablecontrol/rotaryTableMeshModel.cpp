#include "rotaryTableMeshModel.h"

RotaryTableMeshModel::RotaryTableMeshModel()
{
}

Mesh3D RotaryTableMeshModel::getMesh(const CameraLocationAngles &state)
{
    Mesh3D mesh;
    mesh.switchColor(true);
    int stackstate = (int)mesh.transformStack.size();

    mesh.setColor(RGBColor::Red());
    mesh.addCylinder(Vector3dd(0,0,-50), 525, 100, 3, degToRad(30));
    mesh.mulTransform(Matrix33::RotationZ(state.yaw()));

    mesh.setColor(RGBColor::Green());
    mesh.addAOB(Vector3dd(-200,-200, 0), Vector3dd(200, 200, 400));

    mesh.addAOB(Vector3dd(-200, -200, 400), Vector3dd(-150, 200, 400 + 250));
    mesh.addAOB(Vector3dd( 150, -200, 400), Vector3dd( 200, 200, 400 + 250));

    mesh.mulTransform(Matrix44::Shift(0,0,400 + 250));
    mesh.mulTransform(Matrix33::RotationY(M_PI / 2.0));
    mesh.addCylinder(Vector3dd(0, 0, -175), 125, 50, 20);
    mesh.addCylinder(Vector3dd(0, 0,  175), 125, 50, 20);
    mesh.popTransform();

    mesh.mulTransform(Matrix33::RotationX(state.pitch()));

    mesh.setColor(RGBColor::Blue());
    mesh.mulTransform(Matrix33::RotationY(M_PI / 2.0));
    mesh.addCylinder(Vector3dd::Zero(), 100, 300, 20);
    mesh.popTransform();
    //mesh.addAOB(Vector3dd(-150,-150, 0), Vector3dd( 150, 150, 50));
    mesh.addCylinder(Vector3dd(0, 0, 25), 150, 50, 20);

    mesh.mulTransform(Matrix44::Shift(0,0,50));
    mesh.mulTransform(Matrix33::RotationZ(state.roll()));
    mesh.setColor(RGBColor::Yellow());
   // mesh.addAOB(Vector3dd(-150,-150, 0), Vector3dd( 150, 150, 50));
    mesh.addCylinder(Vector3dd(  0, 0, 25), 150, 50, 20);
    mesh.addCylinder(Vector3dd(0, 125, 60),  25, 20, 10);

    while (mesh.transformStack.size() > (size_t)stackstate) {
        mesh.popTransform();
    }

    return mesh;
}
