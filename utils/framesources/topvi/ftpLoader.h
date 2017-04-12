#pragma once

#include "global.h"

using namespace std;

class FtpLoader
{
public:
    bool inited = false;
    string link;
    string outputDir;

    FtpLoader():
        inited(false),
        link("unknown"),
        outputDir("")
    {
    };

    FtpLoader(string _outputDir):
        inited(false),
        link("unknown"),
        outputDir(_outputDir)
    {
    };

    int makeTest();

    int init(string _link);
    void setOutput(string _outputDir);
    int getFile();
};

