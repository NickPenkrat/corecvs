#pragma once

#include "vector2d.h"

#include <opencv2/core/core.hpp>        // cv::Mat, Point2f

using namespace corecvs;

namespace corecvs {
    class SelectableGeometryFeatures;
};

typedef vector<vector<Vector2dd> > Straights;

class OpenCvCheckerboardDetector
{
public:
    OpenCvCheckerboardDetector();
   ~OpenCvCheckerboardDetector();

    enum BoardAlign {
        LEFT,
        RIGHT,
        TOP,
        BOTTOM,
        NONE
    };

    static bool DetectFullCheckerboard(const cv::Mat &mat, int width, int height, SelectableGeometryFeatures *lineList,
                                       int precise = 11,
                                       int maxIterationCount = 100,
                                       double minAccuracy = 0.001);
    static bool DetectFullCheckerboard(const cv::Mat &mat, int width, int height, Straights *straights,
                                       int precise = 11,
                                       int maxIterationCount = 100,
                                       double minAccuracy = 0.001);

    static BoardAlign DetectPartCheckerboardH(const cv::Mat &mat, int width, int height, Straights straights);
    static BoardAlign DetectPartCheckerboardV(const cv::Mat &mat, int width, int height, Straights straights);

    static void DrawCheckerboardLines(cv::Mat &dst, const Straights &straights);

private:
    static void fillStraight(const vector<cv::Point2f> &buffer, int width, int height, Straights *straights);
    static void fillLineList(const Straights &straights, int width, int height, SelectableGeometryFeatures *lineList);
};
