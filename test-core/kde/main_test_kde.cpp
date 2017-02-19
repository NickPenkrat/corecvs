#include <stdio.h>
#include <stdlib.h>
#include "gtest/gtest.h"

#include <stdint.h>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>

#include "kde.h"
#include "vector2d.h"

std::vector<corecvs::Vector2dd> testData{
    {2397.5, 2287.11},
    {2452.63,2284.65},
    {2507.55,2282.62},
    {2563.36,2280.35},
    {2619.12,2278.1 },
    {2675.08,2275.7 },
    {2731.27,2273.4 },
    {2787.69,2271.07},
    {2844.37,2268.69},
    {2901.33,2266.35},
    {2958.46,2263.88},
    {3015.79,2261.53},
    {3073.3, 2259.08},
    {3131.16,2256.62},
    {3189.24,2254.13},
    {3247.44,2251.55},
    {3305.88,2249.08}};

std::vector<double> split(const std::string &s, char delim, std::vector<double> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(atof(item.c_str()));
    }
    return elems;
}

std::vector<double> split(const std::string &s, char delim) {
    std::vector<double> elems;
    split(s, delim, elems);
    return elems;
}

TEST(KDE, testKDE)
{
    std::cout << "Start" << std::endl;
    corecvs::kde* kde = new corecvs::kde;

    for(auto& d: testData){
        kde->addData(d[0],d[1]);
    }

    kde->calcPDF(100,100);

    delete kde;
}
