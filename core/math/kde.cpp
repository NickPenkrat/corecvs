#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>

#include "core/math/kde.h"
#include "core/utils/log.h"
namespace corecvs {


void kde::defaultBandwidth(int currentVariable)
{
    if (!countMap[currentVariable])
    {
        L_ERROR << "No data!";
        exit(1);
    }
    double x = sumXMap[currentVariable] / countMap[currentVariable];
    double x2 = sumX2Map[currentVariable] / countMap[currentVariable];
    double sigma = sqrt(x2 - (x * x));
    double b = sigma * (pow((3.0 * countMap[currentVariable] / 4.0), (-1.0 / 5.0)));

    //L_INFO << "Bandwidth " << currentVariable << " = " << b;

    bandwidthMap[currentVariable] = b;
}

void kde::calcBandwidth()
{
    for (int currentVariable = 0; currentVariable < (int)dataMatrix.size(); currentVariable++)
    {
        if (bandwidthMap[currentVariable] == -1.0)
        {
            defaultBandwidth(currentVariable);
        }
    }
}

double kde::gaussPDF(double x, double m, double s)
{
    double z = (x - m) / s;
    return exp(-0.5 * z * z) / (s * sqrt(2.0 * M_PI));
}

double kde::pdf(double x)
{
    std::vector<double> tmp;
    tmp.push_back(x);
    return pdf(tmp);
}

double kde::pdf(double x, double y)
{
    std::vector<double> tmp;
    tmp.push_back(x);
    tmp.push_back(y);
    return pdf(tmp);
}

double kde::pdf(std::vector<double>& data)
{
    CORE_ASSERT_TRUE_S(dataMatrix.size());

    calcBandwidth();
    double d = 0.0;
    for (size_t i = 0; i < dataMatrix[0].size(); i++)
    {
        double a = 1.0;
        for (size_t currentVariable = 0; currentVariable < dataMatrix.size(); currentVariable++)
        {
            a *= gaussPDF(data[currentVariable], dataMatrix[currentVariable][i], bandwidthMap[(int)currentVariable]);
        }
        d += a;
    }
    return d / countMap[0];
}

void kde::addData(double x)
{
    std::vector<double> tmp;
    tmp.push_back(x);
    addData(tmp);
}

void kde::addData(double x, double y)
{

    std::vector<double> tmp;
    tmp.push_back(x);
    tmp.push_back(y);
    addData(tmp);
}

void kde::addData(std::vector<double>& x)
{
    if (!dataMatrix.size())
    {
        for (int i = 0; i < (int)x.size(); i++)
        {
            std::vector<double> tmp;
            tmp.push_back(x[i]);
            dataMatrix.push_back(tmp);
            sumXMap[i] = x[i];
            sumX2Map[i] = x[i] * x[i];
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
            L_INFO << "Number of variables doesn't match!";
        }
        else
        {
            for (int i = 0; i < (int)x.size(); i++)
            {
                dataMatrix[i].push_back(x[i]);
                sumXMap [i] += x[i];
                sumX2Map[i] += x[i] * x[i];
                countMap[i]++;
                minMap  [i] = std::min(x[i], minMap[i]);
                maxMap  [i] = std::max(x[i], maxMap[i]);
                bandwidthMap[i] = -1.0;
            }
        }
    }
}

std::vector<double> kde::calcPDF(
    int testPointCountX,
    int testPointCountY,
    double passLevel,
    std::vector<double> minXY,
    std::vector<double> maxXY
    )
{
    CORE_ASSERT_TRUE_S(dataMatrix.size());

    double minX = minXY[0];
    double maxX = maxXY[0];
    double minY = minXY[1];
    double maxY = maxXY[1];

    L_INFO << "min x,y = " << minX << "," << minY;
    L_INFO << "max x,y = " << maxX << "," << maxY;

    ndX = testPointCountX;
    ndY = testPointCountY;

    double xIncrement = (maxX - minX) / ndX;
    double yIncrement = (maxY - minY) / ndY;
    double y = minY;
    double x = minX;

    L_INFO << "increment x,y = " << xIncrement << "," << yIncrement;

    calcBandwidth();

    L_INFO << "# bandwidth var 1: " << getBandwidth(0);
    L_INFO << "# bandwidth var 2: " << getBandwidth(1);

    std::vector<double> def;

    double min = 0.0;
    double max = 0.0;

    for (int i = 0; i < ndX; i++)
    {
        x += xIncrement;
        y = minY;
        for (int j = 0; j < ndY; j++)
        {
            y += yIncrement;
            def.emplace_back(pdf(x, y));
            min = def.back() < min ? def.back() : min;
            max = def.back() > max ? def.back() : max;
        }
    }

    int underPass = 0;
    int index = 0;

    for (int i = 0; i < ndX; i++)
    {
        std::ostringstream ss;
        for (int j = 0; j < ndY; j++)
        {
            double norm = (def[index] - min) / (max - min);
            if (norm < passLevel)
                underPass++;
            def[index++] = norm;
            ss << (norm < passLevel ? 0 : 1) << "\t";
        }
        L_INFO << ss.str();
    }

    L_INFO << "Complete";

    return def;
}

std::vector<double> kde::calcPDF(int testPointCountX, int testPointCountY, double passLevel)
{
    CORE_ASSERT_TRUE_S(dataMatrix.size());

    double minX = getMin(0);
    double maxX = getMax(0);
    double minY = getMin(1);
    double maxY = getMax(1);

    return calcPDF(testPointCountX,
                   testPointCountY,
                   passLevel,
                   std::vector<double>({minX, minY}),
                   std::vector<double>({maxX, maxY})
                   );
}

double kde::calcCoverage(int w, int h, int gridSize, double passLevel)
{
    if (!getDataSize() || gridSize <= 0)
        return 0.;

    double wKde = w;
    double hKde = h;
    double s = std::sqrt(wKde * hKde);

    int numX = std::ceil(wKde / s * gridSize) + 1;
    int numY = std::ceil(hKde / s * gridSize) + 1;
    L_INFO_P("image %dx%d is divided onto %dx%d parts", w, h, numX, numY);

    std::vector<double> kdeResult = calcPDF(numX, numY, passLevel
        , std::vector<double>({ 0.0, 0.0 })
        , std::vector<double>({ wKde, hKde }));

    int underPass = 0;
    for (double d : kdeResult)
    {
        if (d < passLevel) underPass++;
    }

    return 100 - 100.0 * underPass / (numX * numY);     // in percent
}

} // namespace corecvs
