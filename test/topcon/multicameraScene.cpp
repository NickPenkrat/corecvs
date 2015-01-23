#include <fstream>
#include <iostream>
#include <cstddef>
#include <string>

#include "multicameraScene.h"

#include "log.h"
#include "vector2d.h"
#include "vector3d.h"
#include "quaternion.h"
#include "matrix33.h"

using std::ifstream;
using std::ios;
using std::string;
using std::endl;

using corecvs::Vector2dd;
using corecvs::Vector3dd;
using corecvs::Quaternion;
using corecvs::Matrix33;

MulticameraScene::MulticameraScene()
{
}

void skipComments(ifstream &s)
{
    while (!s.eof())
    {
        int symbol = s.peek();
        if (isspace(symbol))
        {
            s.get();
            continue;
        }
        if (symbol == '#')
        {
            string comment;
            std::getline(s, comment);
            cout << "Comment" << comment << endl;
            continue;
        }

        break;
    }
}

int MulticameraScene::loadBundlerFile(const std::string &fileName)
{
    ifstream input;
    input.open (fileName.c_str(), ios::in);
    SYNC_PRINT(("MulticameraScene::loadBundlerFile(): Opening: %s\n", fileName.c_str()));
    if (input.fail())
    {
        SYNC_PRINT(("MulticameraScene::loadBundlerFile(): Can't open file\n"));
        return 1;
    }


    int cameraNum = -1;
    /*Ok.. loading*/

    skipComments(input);
    input >> cameraNum;
    SYNC_PRINT(("MulticameraScene::loadBundlerFile(): file has %d cams\n", cameraNum));

    for (int i = 0; i < cameraNum; i++)
    {
        BundlerCamera cam;
        cam.readBundlerCamera(input);
        if (input.bad()) {
            break;
        }
        skipComments(input);

        SYNC_PRINT(("MulticameraScene::loadBundlerFile(): cam %d\n", i));
        cam.print();
        cameraList.push_back(cam);
    }

    for (unsigned i = 0; i < cameraList.size(); i++)
    {
        if (!cameraList[i].checkAsserts())
        {
             SYNC_PRINT(("MulticameraScene::loadBundlerFile(): cam %d failed assert check\n", i));
        }
        cameraList[i].finalizeLoad();
    }
    input.close();
    return 0;
}


int MulticameraScene::loadNVMFile(const std::string &fileName)
{
    ifstream input;
    input.open (fileName.c_str(), ios::in);
    L_INFO_P("Opening: %s", fileName.c_str());
    if (input.fail())
    {
        L_ERROR_P("MulticameraScene::loadNVMFile(): Can't open file");
        return 1;
    }

    string nvmMagicString;
    getline(input, nvmMagicString);

    L_INFO_P("Magic String is %s",nvmMagicString.c_str());

    int cameraNum = -1;
    input >> cameraNum;
    L_INFO_P("File has %d cams", cameraNum);

    for (int i = 0; i < cameraNum; i++)
    {
        BundlerCamera cam;
        cam.readNVMCamera(input);
        if (input.bad()) {
            break;
        }
        L_INFO_P("cam %d\n", i);
        cam.print();
        cameraList.push_back(cam);
    }

    for (unsigned i = 0; i < cameraList.size(); i++)
    {
        cameraList[i].finalizeLoad();
    }

    /* Reading points */
    int pointNum = -1;
    input >> pointNum;
    L_INFO_P("File has %d points", pointNum);
    for (int i = 0; i < pointNum; i++)
    {
        FeaturePoint point;
        point.readFromNVM(input);
        if (input.bad()) {
            break;
        }
        // L_INFO << point.position;
        points.push_back(point);
    }

    L_INFO_P("Points loaded: %d points", points.size());

    input.close();
    return 0;
}

int MulticameraScene::loadInput(const std::string &fileName)
{
    ifstream infile;
    infile.open (fileName.c_str(), ios::in);
    L_INFO_P("Opening: %s", fileName.c_str());
    if (infile.fail())
    {
        L_ERROR_P("Can't open file");
        return 1;
    }

    string nvmMagicString;
    getline(infile, nvmMagicString);

    L_INFO_P("Magic String is %s",nvmMagicString.c_str());
    /* Reading points */
    int pointNum = -1;
    infile >> pointNum;
    L_INFO_P("File has %d points", pointNum);
    for (int i = 0; i < pointNum; i++)
    {
        FeaturePoint point;
        point.readFromInput(infile);
        if (infile.bad()) {
            break;
        }
        L_INFO << point.position;
        this->input.push_back(point);
    }

    L_INFO_P("Points loaded: %d points", input.size());

    infile.close();
    return 0;
}

BundlerCamera::BundlerCamera() :
    focal(0.0),
    optCenter(0.0),
    translation(0.0),
    position(0.0),
    axisAngles(0.0),
    quaternion(0.0, 0.0, 0.0, 1.0),
    rotation(1.0),
    distortion(0.0),
    geographic(0.0)
{

}

int BundlerCamera::readBundlerCamera(std::istream &stream)
{
    stream >> filename;
    stream >> filepath;
    stream >> focal;
    stream >> optCenter;
    stream >> translation;
    stream >> position;
    stream >> axisAngles;

    double t,x,y,z;
    stream >> t >> x >> y >> z;
    quaternion = Quaternion(x,y,z,t);

    stream >> rotation;
    stream >> distortion;
    stream >> geographic;

    return stream.bad();
}

int BundlerCamera::readNVMCamera(std::istream &stream)
{
    stream >> filepath;
    filename = filepath;
    stream >> focal;

    double t,x,y,z;
    stream >> t >> x >> y >> z;
    quaternion = Quaternion(x,y,z,t);

    stream >> translation;
    stream >> distortion;

    double dummy;
    stream >> dummy;

    rotation = quaternion.toMatrix();

    return stream.bad();
}

bool BundlerCamera::checkAsserts()
{
//    SYNC_PRINT(("BundlerCamera::checkAsserts(): called\n"));
    corecvs::Quaternion fromMatrix = corecvs::Quaternion::FromMatrix(rotation);
    if ( (!(quaternion - fromMatrix))  > 0.00001 )
    {
        SYNC_PRINT(("BundlerCamera::checkAsserts(): assert failed\n"));
        cout << "quaternion orig :" << quaternion << endl;
        cout << "quaternion mat  :" << fromMatrix << endl;
        return false;
    }

    Vector3dd t  = translation;
    Vector3dd t1 = -(rotation * position);

    if ( (!(t1 - t))  > 0.00001 )
    {
        SYNC_PRINT(("BundlerCamera::checkAsserts(): assert failed\n"));
        cout << "translation orig:" << t << endl;
        cout << "translation mat :" << t1 << endl;
        return false;
    }

//    SYNC_PRINT(("BundlerCamera::checkAsserts(): exited\n"));
    return true;
}

void BundlerCamera::finalizeLoad()
{
    cameraIntrinsics = CameraIntrinsics(Vector2dd(5397,3598), optCenter, focal, 1.0);
}

void BundlerCamera::print()
{
    cout << "Name:       " << filename << endl;
    cout << "Full name:  " << filepath << endl;
    cout << "Focal len:  " << focal << endl;
    cout << "Opt center: " << optCenter << endl;
    cout << "Trans:      " << translation << endl;
    cout << "Pos:        " << position << endl;
    cout << "Axis angles:" << axisAngles << endl;
    cout << "Q Rotation: " << quaternion << endl;
    cout << "Rotation:   " << endl << rotation << endl;
    cout << "Distortion: " << distortion << endl;
    cout << "Coor:       " << geographic << endl;
}

Ray3d BundlerCamera::rayFromPixel(const Vector2dd &point)
{
 //   Matrix44 Kinv = cameraIntrinsics.getInvKMatrix();
 //  Vector3dd direction = Kinv * Vector3dd(point.x(), point.y(), 1.0);

    //Vector2dd pos((point.x() - optCenter.x()) / focal, (point.y() - optCenter.y()) / focal);
    Vector3dd direction((point.x() - optCenter.x()) / focal, (point.y() - optCenter.y()) / focal, 1.0);




    Ray3d ray(rotation.inv() * direction/* Vector3dd(0,0,1)*/ , translation);
    return ray;
}


int FeaturePoint::readFromNVM(std::istream &stream)
{
    stream >> position;

    int r, g, b;
    stream >> r >> g >> b;
    color = RGBColor(r,g,b);

    int measureCount = 0;
    stream >> measureCount;

    for (int i = 0; i < measureCount; i++ )
    {
        SfmMeasurement measure;
        measure.readFromNVM(stream);
        measurements.push_back(measure);
    }
}

int FeaturePoint::readFromInput(std::istream &stream)
{
    string featureName;

    while (featureName.empty()) {
        getline(stream, featureName);
    }
    L_INFO << "[" << featureName << "]";
    name = featureName;

    int measureCount = 0;
    stream >> measureCount;
    for (int i = 0; i < measureCount; i++ )
    {
        SfmMeasurement measure;
        measure.readFromInput(stream);
        measurements.push_back(measure);
    }
    position = Vector3dd(0);
}


int SfmMeasurement::readFromNVM(std::istream &stream)
{
    stream >> image;
    stream >> feature;
    stream >> coord;

}

int SfmMeasurement::readFromInput(std::istream &stream)
{
    string name;
    stream >> name;
    stream >> image;
    feature = 0;
    stream >> coord;
}
