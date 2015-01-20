#include <fstream>
#include <iostream>
#include <cstddef>
#include <string>

#include "multicameraScene.h"

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

int MulticameraScene::loadBundlerFile(std::string fileName)
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

    for (int i=0; i < cameraNum; i++)
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

    for (int i=0; i < cameraList.size(); i++)
    {
        if (!cameraList[i].checkAsserts())
        {
             SYNC_PRINT(("MulticameraScene::loadBundlerFile(): cam %d failed assert check\n", i));
        }
    }
    input.close();
    return 0;
}

int BundlerCamera::readBundlerCamera(std::istream &stream)
{
   // getline(stream, filename);
   // getline(stream, filepath);
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
    stream >> geometric;

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
    cout << "Coor:       " << geometric << endl;
}
