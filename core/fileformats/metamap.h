#pragma once
/*
    This file contains definitions for MetaData construct which is used in PPMLoader to store custom image metadata
*/
#include <map>

namespace corecvs
{
    class MetaValue : public std::pair<int, double*>
    {
    public:

        MetaValue()
        {
            this->first = 0;
            this->second = nullptr;
        }

        MetaValue(int n, double* val)
        {
            this->first = n;
            this->second = val;
        }

        MetaValue(int n)
        {
            this->first = 1;
            this->second = new double[1];
            this->second[0] = n;
        }

        virtual double& operator[](int n)
        {
            return this->second[n];
        }

        bool operator!()
        {
            return !this->second;
        }

        operator double()
        {
            if (this->first > 0)
                return this->second[0];
            else
                return 0;
        }

        operator int()
        {
            if (this->first > 0)
                return this->second[0];
            else
                return 0;
        }

        explicit operator bool()
        {
            return this->second;
        }
    };

    typedef std::pair<std::string, MetaValue> MetaPair;
    typedef std::map<std::string, MetaValue> MetaData;
}