#ifndef MULTICAMERASCENE_H
#define MULTICAMERASCENE_H

#include <vector>
#include <string>
#include <istream>


#include "cameraParameters.h"
#include "vector2d.h"
#include "vector3d.h"
#include "quaternion.h"
#include "matrix33.h"

using std::vector;
using std::istream;

using corecvs::CameraIntrinsics;

using corecvs::Vector2dd;
using corecvs::Vector3dd;
using corecvs::Quaternion;
using corecvs::Matrix33;

struct BundlerCamera
{
   std::string filename;
   std::string filepath;
   double focal;
   Vector2dd  optCenter;
   Vector3dd translation;
   Vector3dd position;
   Vector3dd axisAngles;
   Quaternion quaternion;
   Matrix33 rotation;
   double distortion;
   Vector3dd geometric;

   int readBundlerCamera(std::istream &stream);

   bool checkAsserts(void);
   void finalizeLoad();

   void print(void);

   CameraIntrinsics cameraIntrinsics;

};


class MulticameraScene
{
public:

    vector<BundlerCamera> cameraList;

    MulticameraScene();

    int loadBundlerFile(std::string fileName);
};

#endif // MULTICAMERASCENE_H
