#pragma once
/*
 * \file cloud.h
 *
 * \date Feb 20, 2013
 */

#include "global.h"
#include "swarmPoint.h"
#include "mesh3d.h"

namespace core3vi {

typedef std::vector<SwarmPoint> CloudBase;


class Cloud : public CloudBase
{
public:
    /*Cloud filtering*/
    Cloud* filterByAABB(const AxisAlignedBox3d &box);
};

} /* namespace core3vi */


/* EOF */
