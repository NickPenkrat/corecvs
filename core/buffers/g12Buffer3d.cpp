#include "g12Buffer3d.h"
namespace core3vi {

SwarmPoint* G12Buffer3d::p0 = NULL;

G12Buffer3d::~G12Buffer3d()
{
    delete m3d;
    delete mSP;
}


} //namespace core3vi
