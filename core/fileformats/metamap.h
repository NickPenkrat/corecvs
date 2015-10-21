#ifndef CMETAMAP_H_
#define CMETAMAP_H_

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

        explicit operator int()
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

        MetaValue& operator -=(MetaValue& other)
        {
            int len = std::min(this->first, other.first);
            for (int i = 0; i < len; i++)
                this->second[i] -= other.second[i];
            return *this;
        }
    };

    typedef std::pair<std::string, MetaValue> MetaPair;
    typedef std::map<std::string, MetaValue> MetaData;
}
#endif