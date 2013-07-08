#ifndef TXTFILTER_H
#define TXTFILTER_H
/**
 * \file txtFilter.h
 * \brief TODO
 *
 * \date Dec 17, 2012
 * \author dpiatkin
 */

#include "filterBlock.h"
#include "filtersCollection.h"

namespace core3vi
{

class TxtFilter : public FilterBlock
{
public:

    TxtFilter(int id = -1) : FilterBlock(id)
    {
        if (instanceId == -1)
             instanceId = ++instanceCounter;
        else instanceCounter = max(instanceCounter, instanceId);

        outputPins.push_back(new TxtPin(this, Pin::OUTPUT_PIN, (id == -1), "0"));
    }

    virtual int getInstanceId() const { return instanceId; }

    virtual const char* getTypeName() const
    {
        return FiltersCollection::typenames[FiltersCollection::TXT_FILTER];
    }

    virtual int operator()(void);

//    TxtParameters mTxtParameters;

    virtual bool setParameters(const void* /*newParameters*/)
    {
//        mTxtParameters = *(TxtParameters*)newParameters;
        return true;
    }

    virtual ~TxtFilter();
private:
    static int instanceCounter;
};

} /* namespace core3vi */
#endif // TXTFILTER_H
