#include <stdio.h>
#include <fstream>
#include <iostream>

#include "global.h"
#include "multicameraScene.h"
#include "mesh3d.h"

using std::ofstream;

int main (int argc, char **argv)
{
    printf("Bundler postreconstrutor.\n");

    if (argc != 2)
    {
        printf("No input\n");
        return -1;
    }

    const char* filename = argv[1];

    SYNC_PRINT(("Starting the read of %s\n", filename));

    MulticameraScene scene;

    scene.loadBundlerFile(filename);

    double scale = 0.1;
    Vector3dd someSize(scale);
    Mesh3D cameraMesh;
    //cameraMesh.addAOB(AxisAlignedBox3d::ByCenter(Vector3dd(0.0,0.0,0.0), someSize));
    cameraMesh.addCamera(scene.cameraList[0].cameraIntrinsics, scale);

    Mesh3D mesh;
    for (int i = 0; i < scene.cameraList.size(); i ++)
    {
        BundlerCamera &cam = scene.cameraList[i];
        Matrix44 trans = Matrix44(cam.rotation, cam.translation);
        Mesh3D campos = cameraMesh.transformed(trans);
        mesh.add(campos);
    }


    ofstream outply;
    outply.open ("out.ply", std::ios::out);
    if (outply.fail())
    {
        SYNC_PRINT(("main: Can't open file\n"));
        return 1;
    }

    mesh.dumpPLY(outply);
    outply.close();







    return 0;
}
