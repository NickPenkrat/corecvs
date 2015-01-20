#include <fstream>
#include <iostream>
#include <cstddef>
#include <string>

#include "multicameraScene.h"

using std::ifstream;
using std::ios;
using std::string;

MulticameraScene::MulticameraScene()
{
}

int MulticameraScene::loadBindlerFile(std::string fileName)
{
    ifstream input;
    input.open (fileName.c_str(), ios::in);
    SYNC_PRINT(("MulticameraScene::loadBindlerFile(): Opening: %s\n", fileName.c_str()));
    if (input.fail())
    {
        SYNC_PRINT(("MulticameraScene::loadBindlerFile(): Can't open file\n"));
        return 1;
    }

    string line;

    int cameraNum = -1;
    /*Ok.. loading*/
    while (!input.eof())
    {
        input >> line;

        size_t startpos = line.find_first_not_of(" \t");
        if( string::npos != startpos )
        {
            line = line.substr( startpos );
        }
        if(line[0] == '#' || line.length() == 0)
        {
            SYNC_PRINT(("Comment:%s", line.c_str()));
        }

        if (cameraNum == -1)
        {
            line >> cameraNum;
        }



    }





    input.close();
    return 0;
}

int BundlerCamera::readBundlerCamera(std::istream &stream)
{
    stream >> filename;
    stream >> optCenter;
    stream >> translation;
    stream >> position;
    stream >> axisAngles;
    stream >> quaternion;
    stream >> rotation;
    stream >> distortion;
    stream >> geometric;

}

void BundlerCamera::print()
{
    cout << filename << endl;
    cout << optCenter << endl;
    cout << translation << endl;
    cout << position << endl;
    cout << axisAngles << endl;
    cout << quaternion << endl;
    cout << rotation << endl;
    cout << distortion << endl;
    cout << geometric << endl;
}
