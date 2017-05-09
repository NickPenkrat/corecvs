#pragma once

#include "global.h"

using namespace std;

class FtpLoader
{
public:
    bool inited = false;
    string addr;
    string link;
    string outputDir;
    string activeFile;

    FtpLoader():
        inited(false),
        addr("ftp://193.232.110.156/"),
        link("unknown"),
        outputDir("")
    {
    };

    FtpLoader(string _outputDir):
        inited(false),
        addr("ftp://193.232.110.156/"),
        link("unknown"),
        outputDir(_outputDir)
    {
    };

    int makeTest();

    int init(string _addr, string _link);
    void setAddr(string _addr);
    void setOutput(string _outputDir);
    int getFile();
};

