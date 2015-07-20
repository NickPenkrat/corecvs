#ifndef OPENCVCHEKERBOARDDETECTOR_H
#define OPENCVCHEKERBOARDDETECTOR_H

#include <opencv/cv.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include "global.h"

#include "selectableGeometryFeatures.h"

using namespace cv;

class OpenCvCheckerboardDetector{
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
    static bool DetectFullCheckerboard(const cv::Mat &mat, int width, int height, vector<vector<Vector2dd> > *straights,
                                       int precise = 11,
                                       int maxIterationCount = 100,
                                       double minAccuracy = 0.001);
    static BoardAlign DetectPartCheckerboardH(const cv::Mat &mat, int width, int height, vector<vector<Vector2dd> > straights);
    static BoardAlign DetectPartCheckerboardV(const cv::Mat &mat, int width, int height, vector<vector<Vector2dd> > straights);
    static void DrawCheckerboardLines(cv::Mat &dst, const vector<vector<Vector2dd> > &straights);
private:
    static void fillStraight(const vector<Point2f> &buffer, int width, int height, vector<vector<Vector2dd> > *straights);
    static void fillLineList(const vector<vector<Vector2dd> > &straights, int width, int height, SelectableGeometryFeatures *lineList);
};

#endif
