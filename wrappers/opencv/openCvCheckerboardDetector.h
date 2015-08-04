#ifndef OPENCVCHEKERBOARDDETECTOR_H
#define OPENCVCHEKERBOARDDETECTOR_H

#include <opencv/cv.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include "global.h"

#include "g8Buffer.h"
#include "checkerboardDetectionParameters.h"
#include "selectableGeometryFeatures.h"

using namespace cv;
using corecvs::G8Buffer;

class CheckerboardDetectionParameters;
namespace corecvs {
    class G8Buffer;
    class SelectableGeometryFeatures;
    class ObservationList;
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

    static bool DetectFullCheckerboard(G8Buffer *input,
                                       const CheckerboardDetectionParameters &params,
                                       SelectableGeometryFeatures *lineList,
                                       G8Buffer **output = NULL);

    static bool DetectPartCheckerboardV(G8Buffer *input,
                                        const CheckerboardDetectionParameters &params,
                                        ObservationList *observationList,
                                        G8Buffer **output = NULL);

    static bool DetectPartCheckerboardH(G8Buffer *input,
                                        const CheckerboardDetectionParameters &params,
                                        ObservationList *observationList,
                                        G8Buffer **output = NULL);

    static bool DetectFullCheckerboard(const cv::Mat &mat, int width, int height, SelectableGeometryFeatures *lineList,
                                       int precise = 11,
                                       int maxIterationCount = 100,
                                       double minAccuracy = 0.001);
    static bool DetectFullCheckerboard(const cv::Mat &mat, int width, int height, Straights *straights,
                                       int precise = 11,
                                       int maxIterationCount = 100,
                                       double minAccuracy = 0.001);

    static std::vector<std::pair<Vector2dd, Vector3dd>> GetPoints(const cv::Mat &mat, int width, int height, double squareSize = 50.0);
    
    static BoardAlign DetectPartCheckerboardH(const cv::Mat &mat, const CheckerboardDetectionParameters &params, ObservationList *observationList);
    static BoardAlign DetectPartCheckerboardV(const cv::Mat &mat, const CheckerboardDetectionParameters &params, ObservationList *observationList);

    static void DrawCheckerboardLines(cv::Mat &dst, const Straights &straights);
    static void DrawCheckerboardIndex(cv::Mat &dst, const vector<cv::Point2f> &pointbuf);
private:
    static void fillStraight(const vector<cv::Point2f> &buffer, int width, int height, Straights *straights);
    static void fillStraight(const vector<cv::Point2f> &buffer, int width, int height, SelectableGeometryFeatures *lineList);

    static void fillPoints(const vector<cv::Point2f> &pointbuf, cv::Size fullSize, cv::Size partSize, cv::Size cellSize, BoardAlign alignment, ObservationList *observationList);

    static bool fastCheckCheckerboard(const cv::Mat &mat, cv::Size boardSize);
};

#endif
