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
#include "rgbColor.h"
#include "line.h"

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
   Vector3dd geographic;

   BundlerCamera();

   int readBundlerCamera(std::istream &stream);
   int readNVMCamera    (std::istream &stream);

   bool checkAsserts(void);
   void finalizeLoad();

   void print(void);

   CameraIntrinsics cameraIntrinsics;

   Ray3d rayFromPixel(const Vector2dd &point);


};

struct SfmMeasurement {
    int image;
    int feature;
    Vector2dd coord;

    int readFromNVM   (std::istream &stream);
    int readFromInput (std::istream &stream);

};

struct FeaturePoint {
   Vector3dd position;
   RGBColor color;
   string name;

   vector<SfmMeasurement> measurements;
   vector<Vector3dd> guesses;

   int readFromNVM   (std::istream &stream);
   int readFromInput (std::istream &stream);
};


class MulticameraScene
{
public:

    vector<BundlerCamera> cameraList;
    vector<FeaturePoint>  points;

    vector<FeaturePoint>  input;


    MulticameraScene();

    int loadBundlerFile(const std::string &fileName);
    int loadNVMFile    (const std::string &fileName);

    int loadInput      (const std::string &fileName);
};


#endif // MULTICAMERASCENE_H
