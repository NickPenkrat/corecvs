/* ===========================================================================
* MAGNA Electronics - C O N F I D E N T I A L
*  This document in its entirety is CONFIDENTIAL and may not be disclosed,
*  disseminated or distributed to parties outside MAGNA Electronics
*  without written permission from MAGNA Electronics.
* ===========================================================================
* SHORT:   spherical_lut.c
* DESIGN:
* DESCRIPTION:
*   This file holds the tables for spherical distortion correction.
*
* ORIGINAL AUTHOR: Werner Kozek
* OTHER AUTHORS: Alexander Pimenov
* CREATED:         Jan 17, 2011
* LAST MODIFIED:   Jan 17, 2011
*
* ======================================================================== */

// #include "basetypes.h"
#include "spherical_lut.h"

extern double WarpToUnwarpLUT_HD[LUT_LEN_HD][2] = {
    { 6.283823374, 8 },
    { 12.57147702, 16.4 },
    { 18.86680054, 24.4 },
    { 25.1736523, 32.4 },
    { 31.49591887, 40.8 },
    { 37.8375247, 48.8 },
    { 44.20244193, 56.8 },
    { 50.59470049, 65.2 },
    { 57.01839852, 73.2 },
    { 63.47771306, 81.2 },
    { 69.97691129, 89.6 },
    { 76.5203622, 97.6 },
    { 83.11254881, 106 },
    { 89.75808102, 114 },
    { 96.46170928, 122.4 },
    { 103.2283389, 130.4 },
    { 110.0630453, 138.8 },
    { 116.9710906, 146.8 },
    { 123.9579408, 155.2 },
    { 131.0292843, 163.6 },
    { 138.1910526, 172 },
    { 145.4494413, 180.4 },
    { 152.8109338, 188.4 },
    { 160.2823267, 196.8 },
    { 167.8707569, 205.2 },
    { 175.5837319, 214 },
    { 183.4291618, 222.4 },
    { 191.4153954, 230.8 },
    { 199.5512585, 239.2 },
    { 207.8460969, 248 },
    { 216.3098228, 256.4 },
    { 224.9529667, 265.2 },
    { 233.7867336, 273.6 },
    { 242.8230661, 282.4 },
    { 252.0747138, 291.2 },
    { 261.5553101, 300 },
    { 271.279458, 308.8 },
    { 281.2628255, 317.6 },
    { 291.522252, 326.8 },
    { 302.0758672, 335.6 },
    { 312.9432256, 344.8 },
    { 324.1454559, 353.6 },
    { 335.705431, 362.8 },
    { 347.6479589, 372 },
    { 360, 381.2 },
    { 372.790913, 390.4 },
    { 386.0527356, 399.6 },
    { 399.8205053, 409.2 },
    { 414.1326266, 418.4 },
    { 429.0312933, 428 },
    { 444.5629764, 437.6 },
    { 460.7789876, 447.2 },
    { 477.7361358, 456.8 },
    { 495.4974914, 466.4 },
    { 514.1332824, 476 },
    { 533.7219487, 486 },
    { 554.351387, 495.6 },
    { 576.1204305, 505.6 },
    { 599.1406136, 515.6 },
    { 623.5382907, 525.6 },
    { 649.4571919, 535.6 },
    { 677.0615275, 546 },
    { 706.539782, 556 },
    { 738.109383, 566.4 },
    { 772.0224914, 576.4 },
    { 808.5732386, 586.8 },
    { 848.1068517, 597.6 },
    { 891.0312672, 608 },
    { 937.8320633, 618.4 },
    { 989.091871, 629.2 },
    { 1045.515916, 640 },
    { 1107.966073, 650.8 },
    { 1177.506943, 661.6 },
    { 1255.4692, 672.8 },
    { 1343.538291, 684 },
    { 1443.881136, 695.2 },
    { 1559.331315, 706.4 },
    { 1693.666839, 718 },
    { 1852.039446, 729.6 },
    { 2041.661455, 741.2 },
    { 2272.950545, 753.6 },
    { 2561.5331, 765.6 },
    { 2931.964714, 777.6 },
    { 3425.171204, 790 },
    { 4114.818829, 802.4 },
    { 5148.239852, 815.2 },
    { 6869.209208, 828 },
    { 10309.05118, 841.2 },
    { 20624.38619, 855.2 },
    { 5.87684E+18, 866 }

};
extern double UnwarpToWarpLUT_HD[LUT_LEN_HD][2] = {
    { 6.283823374, 8 },
    { 12.57147702, 16.4 },
    { 18.86680054, 24.4 },
    { 25.1736523, 32.4 },
    { 31.49591887, 40.8 },
    { 37.8375247, 48.8 },
    { 44.20244193, 56.8 },
    { 50.59470049, 65.2 },
    { 57.01839852, 73.2 },
    { 63.47771306, 81.2 },
    { 69.97691129, 89.6 },
    { 76.5203622, 97.6 },
    { 83.11254881, 106 },
    { 89.75808102, 114 },
    { 96.46170928, 122.4 },
    { 103.2283389, 130.4 },
    { 110.0630453, 138.8 },
    { 116.9710906, 146.8 },
    { 123.9579408, 155.2 },
    { 131.0292843, 163.6 },
    { 138.1910526, 172 },
    { 145.4494413, 180.4 },
    { 152.8109338, 188.4 },
    { 160.2823267, 196.8 },
    { 167.8707569, 205.2 },
    { 175.5837319, 214 },
    { 183.4291618, 222.4 },
    { 191.4153954, 230.8 },
    { 199.5512585, 239.2 },
    { 207.8460969, 248 },
    { 216.3098228, 256.4 },
    { 224.9529667, 265.2 },
    { 233.7867336, 273.6 },
    { 242.8230661, 282.4 },
    { 252.0747138, 291.2 },
    { 261.5553101, 300 },
    { 271.279458, 308.8 },
    { 281.2628255, 317.6 },
    { 291.522252, 326.8 },
    { 302.0758672, 335.6 },
    { 312.9432256, 344.8 },
    { 324.1454559, 353.6 },
    { 335.705431, 362.8 },
    { 347.6479589, 372 },
    { 360, 381.2 },
    { 372.790913, 390.4 },
    { 386.0527356, 399.6 },
    { 399.8205053, 409.2 },
    { 414.1326266, 418.4 },
    { 429.0312933, 428 },
    { 444.5629764, 437.6 },
    { 460.7789876, 447.2 },
    { 477.7361358, 456.8 },
    { 495.4974914, 466.4 },
    { 514.1332824, 476 },
    { 533.7219487, 486 },
    { 554.351387, 495.6 },
    { 576.1204305, 505.6 },
    { 599.1406136, 515.6 },
    { 623.5382907, 525.6 },
    { 649.4571919, 535.6 },
    { 677.0615275, 546 },
    { 706.539782, 556 },
    { 738.109383, 566.4 },
    { 772.0224914, 576.4 },
    { 808.5732386, 586.8 },
    { 848.1068517, 597.6 },
    { 891.0312672, 608 },
    { 937.8320633, 618.4 },
    { 989.091871, 629.2 },
    { 1045.515916, 640 },
    { 1107.966073, 650.8 },
    { 1177.506943, 661.6 },
    { 1255.4692, 672.8 },
    { 1343.538291, 684 },
    { 1443.881136, 695.2 },
    { 1559.331315, 706.4 },
    { 1693.666839, 718 },
    { 1852.039446, 729.6 },
    { 2041.661455, 741.2 },
    { 2272.950545, 753.6 },
    { 2561.5331, 765.6 },
    { 2931.964714, 777.6 },
    { 3425.171204, 790 },
    { 4114.818829, 802.4 },
    { 5148.239852, 815.2 },
    { 6869.209208, 828 },
    { 10309.05118, 841.2 },
    { 20624.38619, 855.2 },
    { 5.87684E+18, 866 }

};

double WarpToUnwarpLUT[LUT_LEN][2] = {
         {0.000000,1.000000},
         {92.045220,0.999853},
         {369.278356,0.999431},
         {834.900028,0.998797},
         {1493.954171,0.998047},
         {2352.938104,0.997301},
         {3419.345507,0.996698},
         {4701.216363,0.996380},
         {6206.768442,0.996481},
         {7944.163436,0.997124},
         {9921.436249,0.998414},
         {12146.592366,1.000435},
         {14627.828484,1.003252},
         {17373.851495,1.006913},
         {20394.232446,1.011453},
         {23699.770151,1.016903},
         {27302.813421,1.023287},
         {31217.535042,1.030635},
         {35460.110003,1.038982},
         {40048.800208,1.048377},
         {45003.902880,1.058885},
         {50347.551183,1.070592},
         {56103.329126,1.083614},
         {62295.687248,1.098101},
         {68949.168431,1.114242},
         {76087.440794,1.132278},
         {83732.228617,1.152503},
         {91902.206244,1.175281},
         {100612.020948,1.201053},
         {109871.552115,1.230356},
         {119685.523953,1.263838},
         {130053.520869,1.302289},
         {140970.391821,1.346678},
         {152426.920080,1.398213},
         {164410.624386,1.458419},
         {176906.552735,1.529269},
         {189897.904168,1.613363},
         {203366.391732,1.714222},
         {217292.279774,1.836745},
         {231654.095520,1.987971},
         {246427.944889,2.178404},
         {261586.465933,2.424443},
         {277097.388139,2.753208},
         {292921.740384,3.212989},
         {309011.642722,3.898899},
         {325307.856518,5.027708},
         {341737.198150,7.223664},
         {358210.138273,13.324111}    };


double UnwarpToWarpLUT[LUT_LEN][2] = {
        {0.000000,1.000000},
        {92.018103,1.000147},
        {368.858400,1.000569},
        {832.893058,1.001204},
        {1488.123721,1.001957},
        {2340.255759,1.002706},
        {3396.804346,1.003313},
        {4667.236628,1.003634},
        {6163.156970,1.003532},
        {7898.535737,1.002884},
        {9889.998848,1.001588},
        {12157.173512,0.999565},
        {14723.126226,0.996758},
        {17614.882422,0.993135},
        {20864.074914,0.988676},
        {24507.728208,0.983378},
        {28589.232227,0.977243},
        {33159.537140,0.970276},
        {38278.636852,0.962480},
        {44017.424691,0.953855},
        {50460.014640,0.944390},
        {57706.684554,0.934063},
        {65877.620356,0.922838},
        {75117.732344,0.910663},
        {85602.896456,0.897471},
        {97548.121614,0.883176},
        {111218.379210,0.867677},
        {126943.072408,0.850861},
        {145135.700162,0.832603},
        {166320.863583,0.812773},
        {191172.008376,0.791241},
        {220565.004568,0.767879},
        {255655.686680,0.742568},
        {297994.392724,0.715199},
        {349699.190619,0.685674},
        {413724.785808,0.653907},
        {494292.693350,0.619823},
        {597603.478294,0.583355},
        {733063.993123,0.544441},
        {915503.912793,0.503025},
        {1169409.861614,0.459052},
        {1537585.847499,0.412466},
        {2100441.484776,0.363213},
        {3023919.418479,0.311237},
        {4697413.174758,0.256483},
        {8223083.381415,0.198898},
        {17832297.206513,0.138434},
        {63593741.644753,0.075052} };
