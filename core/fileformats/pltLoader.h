#ifndef HPGL_LOADER_H
#define HPGL_LOADER_H

#include <string>
#include <vector>

namespace corecvs {

class HPGLProgram
{
public:
    struct Record {
        std::string command;
        std::vector<double> params;
    };

    std::vector<Record>  program;
};


class HPGLLoader
{
public:
    bool trace = true;


    HPGLLoader();
    int loadHPGLcode(std::istream &input, HPGLProgram &program);
};

} // namespace corecvs

#endif // HPGL_LOADER_H
