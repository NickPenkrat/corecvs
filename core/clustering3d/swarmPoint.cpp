#include "swarmPoint.h"

namespace core3vi {

double SwarmPoint::distTo(SwarmPoint other)
{
    return (this->point - other.point).l1Metric();
}

} //namespace core3vi
