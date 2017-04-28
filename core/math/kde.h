#ifndef KDE_H_
#define KDE_H_

/**
 * \file kde.h
 * \brief kde
 *
 * \date 
 */
#include <vector>
#include <map>

#include "global.h"

namespace corecvs {

class RGB24Buffer;


class kde
{
	public:
        void    addData(double x);
        void    addData(double x, double y);
        void    addData(std::vector<double>& x);
        size_t  getDataSize() {return dataMatrix.size();}

        std::vector<double>
                calcPDF(int testPointCountX = 10, int testPointCountY = 10, double passLevel = .01);

        std::vector<double>
                calcPDF(
                int testPointCountX,
                int testPointCountY,
                double passLevel,
                std::vector<double> min,
                std::vector<double> max
                );

        double  calcCoverage(int w, int h, int gridSize = 10, double passLevel = 0.01);

    private:
        std::vector<std::vector<double> >   dataMatrix;

        std::map<int,double>
                sumXMap,
                sumX2Map,
                countMap,
                minMap,
                maxMap,
                bandwidthMap;

        int     extension;
        int     ndX, ndY;

        double  getMin(int x)       { return minMap[x]; }
        double  getMax(int x)       { return maxMap[x]; }
        double  getBandwidth(int x) { return(bandwidthMap[x]); }

        void    calcBandwidth();
        void    defaultBandwidth(int currentVariable);
        double  gaussPDF(double x, double m, double s);
        double  pdf(double x);
        double  pdf(double x, double y);
        double  pdf(std::vector<double>& data);
};

} //namespace corecvs

#endif  //KDE_H_
