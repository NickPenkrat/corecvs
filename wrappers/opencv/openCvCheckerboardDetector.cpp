#include <opencv/cv.h>
#include "OpenCVTools.h"
#include "openCvCheckerboardDetector.h"

void OpenCvCheckerboardDetector::DrawCheckerboardLines(cv::Mat &dst, const vector<vector<Vector2dd> > &straights){
    for(int i = 0; i < straights.size(); i++)
    {
        double prevX = 0;
        double prevY = 0;
        for(int p = 0; p < straights[i].size(); p++)
        {
            double curX = straights[i][p].x();
            double curY = straights[i][p].y();
            if (prevX != 0 && prevY != 0)
            {
                cv::line(dst, Point(prevX, prevY), Point(curX, curY), Scalar(0,255,0));
            }
            prevX = curX;
            prevY = curY;
        }
    }
}

bool OpenCvCheckerboardDetector::DetectFullCheckerboard(const cv::Mat &mat, int width, int height, vector<vector<Vector2dd> >* straights){
    int             found;
    Size            boardSize(width, height);
    vector<Point2f>   pointbuf;
    found = findChessboardCorners( mat, boardSize, pointbuf, CV_CALIB_CB_ADAPTIVE_THRESH );
    if(!found){
          SYNC_PRINT(("Failed to detect.\n"));
        return 0;
    }
    fillStraight(pointbuf, width, height, straights);
    return found;
}

OpenCvCheckerboardDetector::BoardAlign OpenCvCheckerboardDetector::DetectPartCheckerboardH(const cv::Mat &mat, int width, int height, vector<vector<Vector2dd> >* straights){
    int found;
    vector<Point2f> pointbuf;
    int widthOfPart = 0;
    for(int i = width; i > 2; i--)
    {
        Size boardSize(i, height);
        SYNC_PRINT(("Try %ix%i",i, height));
        DWORD startTime = GetTickCount();
        found = findChessboardCorners( mat, boardSize, pointbuf, CV_CALIB_CB_ADAPTIVE_THRESH );
        if(found)
        {
            fillStraight(pointbuf, width, height, straights);
            widthOfPart = i;
            double left = pointbuf.at(0).x;
            double right = pointbuf.at(width - 1).x;
            for(int j = 1; j < height; j++)
            {
                if(left   > pointbuf.at(width * j).x)
                    left  = pointbuf.at(width * j).x;
                if(right  > pointbuf.at(width * (j + 1) - 1).x)
                    right = pointbuf.at(width * (j + 1) - 1).x;
            }
            if(left < right)
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
        DWORD intervalTime = GetTickCount() - startTime;
        SYNC_PRINT((" failed. Time: %i\n",intervalTime));
    }
    return BoardAlign::NONE;
}

OpenCvCheckerboardDetector::BoardAlign OpenCvCheckerboardDetector::DetectPartCheckerboardV(const cv::Mat &mat, int width, int height, vector<vector<Vector2dd> >* straights){
    int found;
    vector<Point2f> pointbuf;
    int heightOfPart = 0;
    for(int j = height; j > 2; j--)
    {
        Size boardSize(width, j);
        SYNC_PRINT(("Try %ix%i",width, j));
        DWORD startTime = GetTickCount();
        found = findChessboardCorners( mat, boardSize, pointbuf, CV_CALIB_CB_ADAPTIVE_THRESH );
        if(found)
        {
            fillStraight(pointbuf, width, height, straights);
            heightOfPart = j;
            double top = pointbuf.at(0).y;
            double bottom = pointbuf.at(width - 1).y;
            for(int i = 1; i < width; i++)
            {
                if(top     > pointbuf.at(height * i).y)
                    top    = pointbuf.at(height * i).y;
                if(bottom  > pointbuf.at(height * (i + 1) - 1).y)
                    bottom = pointbuf.at(height * (i + 1) - 1).y;
            }
            if(top < bottom)
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
        DWORD intervalTime = GetTickCount() - startTime;
        SYNC_PRINT((" failed. Time: %i\n",intervalTime));
    }
    return BoardAlign::NONE;
}

void OpenCvCheckerboardDetector::fillStraight(const vector<Point2f> &buffer, int width, int height, vector<vector<Vector2dd> >* straights)
{
    straights->clear();
    for (int ih = 0; ih < height; ih++)
    {
        vector<Vector2dd> straight;
        for (int iw = 0; iw < width; iw++)
        {
            straight.push_back(Vector2dd(buffer.at(ih * width + iw).x,buffer.at(ih * width + iw).y));
        }
        straights->push_back(straight);
    }
    for (int iw = 0; iw < width; iw++)
    {
        vector<Vector2dd> straight;
        for (int ih = 0; ih < height; ih++)
        {
            straight.push_back(Vector2dd(buffer.at(iw + width * ih).x,buffer.at(iw + width * ih).y));
        }
        straights->push_back(straight);
    }
}
