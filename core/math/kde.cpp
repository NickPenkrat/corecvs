#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>

#include "kde.h"
#include "rgb24Buffer.h"
#include "abstractPainter.h"

void corecvs::kde::defaultBandwidth(int currentVariable)
{
    if (!countMap[currentVariable])
    {
        std::cout << "No data!" << std::endl;
        exit(1);
    }
    double x  = sumXMap [currentVariable]/countMap[currentVariable];
    double x2 = sumX2Map[currentVariable]/countMap[currentVariable];
    double sigma = sqrt(x2 - (x*x));
    double b = sigma*(pow((3.0*countMap[currentVariable]/4.0),(-1.0/5.0)));

    std::cout << "Bandwidth " << currentVariable << " = " << b << std::endl;

    bandwidthMap[currentVariable] = b;
}

void corecvs::kde::calcBandwidth()
{
    for (int currentVariable = 0; currentVariable < dataMatrix.size(); currentVariable++)
    {
        if (bandwidthMap[currentVariable] == -1.0)
        {
            defaultBandwidth(currentVariable);
        }
    }
}

double corecvs::kde::gaussPDF(double x, double m, double s)
{
    double z = (x - m) / s;
    return exp(-0.5 * z * z) / (s * sqrt(2.0 * M_PI));
}

double corecvs::kde::pdf(double x)
{
    std::vector<double> tmp;
    tmp.push_back(x);
    return(pdf(tmp));
}

double corecvs::kde::pdf(double x, double y)
{
    std::vector<double> tmp;
    tmp.push_back(x);
    tmp.push_back(y);
    return(pdf(tmp));
}

double corecvs::kde::pdf(std::vector<double>& data)
{
    calcBandwidth();
    double d = 0.0;
    for (int i = 0; i < dataMatrix[0].size(); i++)
    {
        double a = 1.0;
        for (int currentVariable = 0; currentVariable < dataMatrix.size(); currentVariable++)
        {
            a *= gaussPDF(data[currentVariable], dataMatrix[currentVariable][i], bandwidthMap[currentVariable]);
        }
        d += a;
    }
    return d / countMap[0];
}

void corecvs::kde::addData(double x)
{
    std::vector<double> tmp;
    tmp.push_back(x);
    addData(tmp);
}

void corecvs::kde::addData(double x, double y)
{
    std::vector<double> tmp;
    tmp.push_back(x);
    tmp.push_back(y);
    addData(tmp);
}

void corecvs::kde::addData(std::vector<double>& x)
{
    if (!dataMatrix.size())
    {
        for (int i = 0; i < x.size(); i++)
        {
            std::vector<double> tmp;
            tmp.push_back(x[i]);
            dataMatrix.push_back(tmp);
            sumXMap[i] = x[i];
            sumX2Map[i] = x[i]*x[i];
            countMap[i] = 1;
            minMap[i] = x[i];
            maxMap[i] = x[i];
            bandwidthMap[i] = -1.0;
        }
    }
    else
    {
        if (x.size() != dataMatrix.size())
        {
            std::cout << "Number of variables doesn't match!" << std::endl;
        }
        else
        {
            for (int i = 0; i < x.size(); i++)
            {
                dataMatrix[i].push_back(x[i]);
                sumXMap[i] += x[i];
                sumX2Map[i] += x[i]*x[i];
                countMap[i]++;
                minMap[i] = x[i] < minMap[i] ? x[i] : minMap[i];
                maxMap[i] = x[i] > maxMap[i] ? x[i] : maxMap[i];
                bandwidthMap[i] = -1.0;
            }
        }
    }
}

std::vector<double> corecvs::kde::calcPDF(int testPointCountX
    , int testPointCountY
    , double passLevel
    , std::vector<double> minXY
    , std::vector<double> maxXY
    , corecvs::RGB24Buffer* buffer
    )
{
    double minX = minXY[0];
    double maxX = maxXY[0];
    double minY = minXY[1];
    double maxY = maxXY[1];

    std::cout << "min x,y = " << minX << "," << minY << std::endl;
    std::cout << "max x,y = " << maxX << "," << maxY << std::endl;

    ndX = testPointCountX;
    ndY = testPointCountY;

    double xIncrement = (maxX - minX) / ndX;
    double yIncrement = (maxY - minY) / ndY;
    double y = minY;
    double x = minX;

    std::cout << "increment x,y = " << xIncrement << "," << yIncrement << std::endl;

    calcBandwidth();

    std::cout << "# bandwidth var 1: " << getBandwidth(0) << std::endl;
    std::cout << "# bandwidth var 2: " << getBandwidth(1) << std::endl;

    std::vector<double> def;

    double min = 0.0;
    double max = 0.0;

    for(int i = 0; i < ndX; i++)
    {
        x += xIncrement;
        y = minY;
        for (int j = 0; j < ndY; j++)
        {
            y += yIncrement;
            def.emplace_back(pdf(x,y));
            min = def.back() < min ? def.back() : min;
            max = def.back() > max ? def.back() : max;
        }
    }

    int underPass = 0;
    int index = 0;

    y = minY;
    x = minX;

    AbstractPainter<RGB24Buffer> painter(buffer);

    for (int i = 0; i < ndX; i++)
    {
        x += xIncrement;
        y = minY;
        for (int j = 0; j < ndY; j++)
        {
            y += yIncrement;
            index++;
            double norm = (def[index] - min)/(max-min);
            if (norm < passLevel)
                underPass++;
            def[index] = norm;
            std::cout << norm << "\t" ;

            painter.drawFormat(x,  y, RGBColor(0x00FF00), 1, std::to_string(norm).c_str());
        }
        std::cout << std::endl;
    }

    std::cout << "Complete " << std::endl;
    return def;
}

std::vector<double> corecvs::kde::calcPDF(int testPointCountX
    , int testPointCountY
    , double passLevel
    , corecvs::RGB24Buffer* buffer)
{
    double minX = getMin(0);
    double maxX = getMax(0);
    double minY = getMin(1);
    double maxY = getMax(1);

    return calcPDF(testPointCountX,
                   testPointCountY,
                   passLevel,
                   std::vector<double>({minX, minY}),
                   std::vector<double>({maxX, maxY}),
                   buffer
                   );
}
