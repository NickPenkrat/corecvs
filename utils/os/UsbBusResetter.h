#ifndef USBBUSRESETTER_H
#define USBBUSRESETTER_H

#include <vector>

class UsbBusResetter
{
public:
    UsbBusResetter();

    virtual std::vector<int> getBusIds()
    {
        return std::vector<int>();
    }
    virtual bool powerOffBus(int /*id*/)
    {
        return false;
    }
    virtual bool powerOnBus(int /*id*/)
    {
        return false;
    }

};

class LinuxUsbResetter : public UsbBusResetter
{


};

#endif // USBBUSRESETTER_H
