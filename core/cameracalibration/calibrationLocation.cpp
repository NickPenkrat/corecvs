#include "mathUtils.h"
#include "calibrationLocation.h"
#include "printerVisitor.h"

namespace corecvs {

using std::endl;

void LocationData::prettyPrint(std::ostream &out)
{
    PrinterVisitor visitor(3, 3, out);
    accept(visitor);
}

void LocationData::prettyPrint1(std::ostream &out)
{
    Quaternion o = orientation.normalised();
    Vector3dd axis = o.getAxis();
    double   angle = radToDeg(o.getAngle());

    out << "Pos:" <<  position << endl;
    out << "Rotation around" << axis << " angle " << angle << "deg" << endl;
}

} // namespace corecvs
