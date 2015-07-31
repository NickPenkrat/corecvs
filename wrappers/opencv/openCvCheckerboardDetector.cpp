#include "openCvCheckerboardDetector.h"
#include "selectableGeometryFeatures.h"
#include "checkerboardDetectionParameters.h"
#include "preciseTimer.h"
/// OpenCV wrapper
#include "OpenCVTools.h"

#include <opencv2/imgproc/imgproc.hpp>  // cornerSubPix
#include <opencv2/calib3d/calib3d.hpp>  // findChessboardCorners

void OpenCvCheckerboardDetector::DrawCheckerboardLines(cv::Mat &dst, const Straights &straights)
{
    for (unsigned i = 0; i < straights.size(); i++)
    {
        double prevX = 0;
        double prevY = 0;
        for (unsigned p = 0; p < straights[i].size(); p++)
        {
            double curX = straights[i][p].x();
            double curY = straights[i][p].y();
            if (prevX != 0 && prevY != 0)
            {
                cv::line(dst, cv::Point(prevX, prevY), cv::Point(curX, curY), cv::Scalar(0,255,0));
            }
            prevX = curX;
            prevY = curY;
        }
    }
}

bool OpenCvCheckerboardDetector::DetectFullCheckerboard(
    G8Buffer *input
  , const CheckerboardDetectionParameters &params
  , SelectableGeometryFeatures *lineList
  , G8Buffer **output
    )
{
    IplImage *iplImage = OpenCVTools::getCVImageFromG8Buffer(input);
    cv::Mat view = cv::Mat(iplImage);

    bool toReturn = DetectFullCheckerboard(view,
                     params.hCrossesCount(),
                     params.vCrossesCount(),
                     lineList,
                     params.preciseDiameter(),
                     params.iterationCount(),
                     params.minAccuracy());

    if (output != NULL)
    {
        G8Buffer *header = G8Buffer::CreateBuffer<G8Buffer>(iplImage->height, iplImage->width, iplImage->widthStep, (uint8_t *)iplImage->imageData);
        *output = new G8Buffer(header);
        delete header;
    }

    cvReleaseImage(&iplImage);
    return toReturn;
}

bool OpenCvCheckerboardDetector::DetectFullCheckerboard(
      const cv::Mat &mat
    , int width, int height
    , SelectableGeometryFeatures *lineList
    , int precise
    , int maxIterationCount
    , double minAccuracy)
{
    CORE_UNUSED(precise);
    CORE_UNUSED(maxIterationCount);
    CORE_UNUSED(minAccuracy);

//    Straights straights;
//    bool detected = DetectFullCheckerboard(mat, width, height, &straights, precise, maxIterationCount, minAccuracy);

    int                 found;
    cv::Size            boardSize(width, height);
    vector<cv::Point2f> pointbuf;

    SYNC_PRINT(("Start ...\n"));
    found = findChessboardCorners(mat, boardSize, pointbuf,
                                  cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK);
    if (!found)
    {
        SYNC_PRINT(("Failed to detect.\n"));
        return 0;
    }

    for (int i = 0; i < height; i++)
    {
        SelectableGeometryFeatures::VertexPath * path = lineList->appendNewPath();
        for (int j = 0; j < width; j++)
        {
            Vector2dd point(pointbuf.at(i * width + j).x, pointbuf.at(i * width + j).y);
            lineList->addVertexToPath(lineList->appendNewVertex(point), path);
        }
    }

    for (int j = 0; j < width; j++)
    {
        SelectableGeometryFeatures::VertexPath * path = lineList->appendNewPath();
        for (int i = 0; i < height; i++)
        {
            Vector2dd point(pointbuf.at(i * width + j).x, pointbuf.at(i * width + j).y);
            lineList->addVertexToPath(lineList->appendNewVertex(point), path);
        }
    }

    return 1;
}


bool OpenCvCheckerboardDetector::DetectFullCheckerboard(
    const cv::Mat &mat
    , int width, int height
    , Straights *straights
    , int precise
    , int maxIterationCount
    , double minAccuracy)
{
    int                 found;
    cv::Size            boardSize(width, height);
    vector<cv::Point2f> pointbuf;
    SYNC_PRINT(("Start ...\n"));
    found = findChessboardCorners(mat, boardSize, pointbuf, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK);
    if (!found)
    {
        SYNC_PRINT(("Failed to detect.\n"));
        return 0;
    }

    for (unsigned i = 0; i < pointbuf.size(); i++)
    {
        SYNC_PRINT(("Point %f %f\n", pointbuf[i].x, pointbuf[i].y));
    }

    SYNC_PRINT(("Finding lines %ix%i...", width, height));
    fillStraight(pointbuf, width, height, straights);
    SYNC_PRINT(("Good\n"));

    if (found && precise)
    {
        cv::cornerSubPix(mat, pointbuf, cv::Size(precise, precise), cv::Size(-1, -1)
            , cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, maxIterationCount, minAccuracy));
    }
    return found;
}

OpenCvCheckerboardDetector::BoardAlign OpenCvCheckerboardDetector::DetectPartCheckerboardH(
    const cv::Mat &mat
    , int width, int height
    , Straights straights)
{
    int found;
    vector<cv::Point2f> pointbuf;
    int widthOfPart = 0;
    for (unsigned i = width; i > 2; i--)
    {
        cv::Size boardSize(i, height);
        SYNC_PRINT(("Try %ix%i",i, height));
        PreciseTimer timer = PreciseTimer::currentTime();

        found = findChessboardCorners(mat, boardSize, pointbuf, CV_CALIB_CB_ADAPTIVE_THRESH);
        if (found)
        {
//            fillStraight(pointbuf, width, height, straights);
            widthOfPart = i;
            double left = pointbuf.at(0).x;
            double right = pointbuf.at(width - 1).x;
            for (unsigned j = 1; j < (unsigned)height; j++)
            {
                if (left  > pointbuf.at(width * j).x)
                    left  = pointbuf.at(width * j).x;
                if (right > pointbuf.at(width * (j + 1) - 1).x)
                    right = pointbuf.at(width * (j + 1) - 1).x;
            }
            if (left < right)
            {
                SYNC_PRINT((" LEFT\n"));
                return BoardAlign::LEFT;
            }
            else
            {
                SYNC_PRINT((" RIGHT\n"));
                return BoardAlign::RIGHT;
            }
        }
        SYNC_PRINT((" failed. Time: %PRIu64 us\n", timer.usecsToNow()));
    }
    return BoardAlign::NONE;
}

OpenCvCheckerboardDetector::BoardAlign OpenCvCheckerboardDetector::DetectPartCheckerboardV(
    const cv::Mat &mat
    , int width, int height
    , Straights straights)
{
    vector<cv::Point2f> pointbuf;
    int heightOfPart = 0, found;

    for (unsigned j = height; j > 2; j--)
    {
        cv::Size boardSize(width, j);
        SYNC_PRINT(("Try %ix%i",width, j));
        PreciseTimer timer = PreciseTimer::currentTime();

        found = findChessboardCorners(mat, boardSize, pointbuf, CV_CALIB_CB_ADAPTIVE_THRESH);
        if (found)
        {
//            fillStraight(pointbuf, width, height, straights);
            heightOfPart = j;
            double top = pointbuf.at(0).y;
            double bottom = pointbuf.at((heightOfPart - 1) *width).y;
            for (unsigned i = 1; i < (unsigned)width; i++)
            {
                if(top     > pointbuf.at(i).y)
                    top    = pointbuf.at(i).y;
                if(bottom  > pointbuf.at((heightOfPart - 1) * width + i).y)
                    bottom = pointbuf.at((heightOfPart - 1) * width + i).y;
            }
            if (top < bottom)
            {
                SYNC_PRINT((" TOP\n"));
                return BoardAlign::TOP;
            }
            else
            {
                SYNC_PRINT((" BOTTOM\n"));
                return BoardAlign::BOTTOM;
            }
        }

        SYNC_PRINT((" failed. Time: %i us\n", timer.usecsToNow()));
    }
    return BoardAlign::NONE;
}

// TODO: reduce copypaste
std::vector<std::pair<Vector2dd, Vector3dd>> OpenCvCheckerboardDetector::GetPoints(
        const cv::Mat &mat,
        int width, int height, double squareSize)
{
    std::vector<std::pair<Vector2dd, Vector3dd>> res;
    vector<cv::Point2f> pointbuf;
    int heightOfPart = 0, found;

    for (unsigned j = height; j > 2; j--)
    {
        cv::Size boardSize(width, j);
        SYNC_PRINT(("Try %ix%i",width, j));
        PreciseTimer timer = PreciseTimer::currentTime();

        found = findChessboardCorners(mat, boardSize, pointbuf, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK);
        if (found)
        {
            heightOfPart = j;
            double mean = 0.0;
            for(auto pt: pointbuf)
                mean += pt.y;
            mean /= pointbuf.size();

            for(int ii = 0; ii < j; ++ii) {
                std::sort(pointbuf.begin() + width * ii, pointbuf.begin() + width * (ii+1), [](const cv::Point2f &a, const cv::Point2f &b) { return a.x < b.x; });
            }
            std::vector<double> yb;
            for(int ii = 0; ii < j; ++ii)
                yb.push_back(pointbuf[ii * width].y);
            std::sort(yb.begin(), yb.end());

            decltype(pointbuf) pbf = pointbuf;
            for(int ii = 0; ii < j; ++ii)
            {
                int jj = 0;
                for(jj = 0; jj < j; ++jj)
                {
                    if(yb[ii] == pointbuf[jj * width].y)
                        break;
                }
                for(int kk = 0; kk < width; ++kk)
                {
                    pbf[ii * width + kk] = pointbuf[jj * width + kk];
                }
            }
            pointbuf = pbf;
            cv::Mat gr;
            cv::cvtColor(mat, gr, CV_BGR2GRAY);
            cv::cornerSubPix(gr, pointbuf, cv::Size(11, 11), cv::Size(-1, -1)
            , cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 1e-3));
            if  (mean < (mat.rows / 2.0))
            {
                SYNC_PRINT((" TOP\n"));
                res.resize(pointbuf.size());
                int idx = 0;
                for(int y = height - j; y <  height; ++y) {
                    for(int x = 0; x < width; ++x) {
                        res[idx].first = Vector2dd(pointbuf[idx].x, pointbuf[idx].y);
                        res[idx].second = Vector3dd(x * squareSize, y * squareSize, 0.0);
                        idx++;
                    }
                }
            }
            else
            {
                SYNC_PRINT((" BOTTOM\n"));
                res.resize(pointbuf.size());
                int idx = 0;
                for(int y = 0; y < j; ++y) {
                    for(int x = 0; x < width; ++x) {
                        res[idx].first = Vector2dd(pointbuf[idx].x, pointbuf[idx].y);
                        res[idx].second = Vector3dd(x * squareSize, y * squareSize, 0.0);
                        idx++;
                    }
                }
            }
            break;
        }

        SYNC_PRINT((" failed. Time: %i us\n", timer.usecsToNow()));
    }
    return res;
}

void OpenCvCheckerboardDetector::fillStraight(
    const vector<cv::Point2f> &buffer
    , int width, int height
    , Straights *straights)
{
    SYNC_PRINT(("--------- %i --------------\n", buffer.at(1).x));
    //Straights tempSstraights;

    for (unsigned ih = 0; ih < (unsigned)height; ih++)
    {
        vector<Vector2dd> straight;
        for (unsigned iw = 0; iw < (unsigned)width; iw++)
        {
            straight.push_back(Vector2dd(buffer.at(ih * width + iw).x
                                       , buffer.at(ih * width + iw).y));
        }
        straights->push_back(straight);
    }
    for (unsigned iw = 0; iw < (unsigned)width; iw++)
    {
        vector<Vector2dd> straight;
        for (unsigned ih = 0; ih < (unsigned)height; ih++)
        {
            straight.push_back(Vector2dd(buffer.at(iw + width * ih).x
                                       , buffer.at(iw + width * ih).y));
        }
        straights->push_back(straight);
    }
    //Mat(tempSstraights).copyTo(straights);
}
