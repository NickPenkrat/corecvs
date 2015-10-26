/**
 * \file    fileformats\metamap.h
 *
 * Declares the metamap for metadata handling.
 */

#ifndef CMETAMAP_H_
#define CMETAMAP_H_

#include "global.h"
#include <map>
#include <string>
#include <vector>

namespace corecvs
{
    typedef std::vector<double> MetaValue;
    typedef std::pair<std::string, MetaValue> MetaPair;
    typedef std::map<std::string, MetaValue> MetaData;
}
#endif