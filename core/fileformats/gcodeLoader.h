#ifndef GCODELOADER_H
#define GCODELOADER_H

#include <iostream>

#include "global.h"

#include "mesh3d.h"

namespace corecvs {

class GcodeLoader
{
public:
    GcodeLoader();
    bool trace = true;
    int arcSteps = 10;

    int loadGcode(istream &input, Mesh3D &mesh);
//    int saveGcode(ostream &out, Mesh3D &mesh);

    virtual ~GcodeLoader();

};

} // namespace corecvs

#endif // GCODELOADER_H
