#pragma once

#include <vector>
#include "global.h"

using namespace std;

class FtpLoader
{
public:
    bool inited = false;
    string addr;
    int linksNumber;
    vector<std::string> links;
    string outputDir;
    string activeFile;

    FtpLoader():
        inited(false),
        addr("ftp://217.71.226.214:3421/"),
        linksNumber(1),
        outputDir("")
    {
    };

    FtpLoader(string _outputDir):
        inited(false),
        addr("ftp://217.71.226.214:3421/"),
        linksNumber(1),
        outputDir(_outputDir)
    {
    };

    int makeTest();

    int init(string _addr, string _links);
    void setAddr(string _addr);
    void setOutput(string _outputDir);

    int getFile(string link);
};

