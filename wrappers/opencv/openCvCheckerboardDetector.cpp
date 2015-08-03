#include <opencv/cv.h>
#include <string>

#include "preciseTimer.h"
#include "OpenCVTools.h"

#include "openCvCheckerboardDetector.h"


void OpenCvCheckerboardDetector::DrawCheckerboardLines(cv::Mat &dst, const vector<vector<Vector2dd> > &straights){
    for(unsigned i = 0; i < straights.size(); i++)
    {
        double prevX = 0;
        double prevY = 0;
        for(unsigned p = 0; p < straights[i].size(); p++)
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

void OpenCvCheckerboardDetector::DrawCheckerboardIndex(cv::Mat &dst, const vector<Point2f> &pointbuf){
    for(unsigned i = 0; i < pointbuf.size(); i++)
    {
        Point2f p1(pointbuf[i]);
        Scalar crossColor = Scalar(255, 50, 50);
        circle (dst, p1, 22, crossColor, 1);
        line(dst, p1 + Point2f(-18, -18), p1 + Point2f(-9, -9), crossColor);
        line(dst, p1 + Point2f( 18,  18), p1 + Point2f( 9,  9), crossColor);
        line(dst, p1 + Point2f( 18, -18), p1 + Point2f( 9, -9), crossColor);
        line(dst, p1 + Point2f(-18,  18), p1 + Point2f(-9,  9), crossColor);

        std::stringstream ss; ss << i;
        putText(dst, ss.str(), Point2f(p1.x + 5, p1.y - 5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0),2);
        putText(dst, ss.str(), Point2f(p1.x + 5, p1.y - 5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255),1);
    }
}

bool OpenCvCheckerboardDetector::DetectFullCheckerboard(
        G8Buffer *input,
        const CheckerboardDetectionParameters &params,
        SelectableGeometryFeatures *lineList,
        G8Buffer **output
        )
{
    IplImage *iplImage = OpenCVTools::getCVImageFromG8Buffer(input);
    Mat view = cv::Mat(iplImage);

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

bool OpenCvCheckerboardDetector::DetectPartCheckerboardV(
        G8Buffer *input,
        const CheckerboardDetectionParameters &params,
        ObservationList *observation,
        G8Buffer **output
        )
{
    IplImage *iplImage = OpenCVTools::getCVImageFromG8Buffer(input);
    Mat view = cv::Mat(iplImage);

    BoardAlign alignment = DetectPartCheckerboardV(view,
                                                   params,
                                                   observation);

    if (output != NULL)
    {
        G8Buffer *header = G8Buffer::CreateBuffer<G8Buffer>(iplImage->height, iplImage->width, iplImage->widthStep, (uint8_t *)iplImage->imageData);
        *output = new G8Buffer(header);
        delete header;
    }
    cvReleaseImage(&iplImage);
    return false;
}

bool OpenCvCheckerboardDetector::DetectPartCheckerboardH(
        G8Buffer *input,
        const CheckerboardDetectionParameters &params,
        ObservationList *observation,
        G8Buffer **output
        )
{
//    IplImage *iplImage = OpenCVTools::getCVImageFromG8Buffer(input);
//    Mat view = cv::Mat(iplImage);

//    bool toReturn = DetectFullCheckerboard(view,
//                     params.hCrossesCount(),
//                     params.vCrossesCount(),
//                     lineList,
//                     params.preciseDiameter(),
//                     params.iterationCount(),
//                     params.minAccuracy());

//    if (output != NULL)
//    {
//        G8Buffer *header = G8Buffer::CreateBuffer<G8Buffer>(iplImage->height, iplImage->width, iplImage->widthStep, (uint8_t *)iplImage->imageData);
//        *output = new G8Buffer(header);
//        delete header;
//    }
//    cvReleaseImage(&iplImage);
    return 0;//toReturn;
}

bool OpenCvCheckerboardDetector::DetectFullCheckerboard(const cv::Mat &mat, int width, int height, SelectableGeometryFeatures *lineList, int precise,
                                                        int maxIterationCount,
                                                        double minAccuracy)
{
    int             found;
    Size            boardSize(width, height);
    vector<Point2f> pointbuf;
    SYNC_PRINT(("Start ...\n"));
    found = findChessboardCorners(mat, boardSize, pointbuf, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK );
    if(found && precise){
        cornerSubPix ( mat ,  pointbuf ,  Size ( precise ,  precise ),  Size ( - 1 ,  - 1 ),
          TermCriteria ( CV_TERMCRIT_EPS  +  CV_TERMCRIT_ITER ,  maxIterationCount, minAccuracy ));
    }
    fillStraight(pointbuf, width, height, lineList);
    return 1;
}

bool OpenCvCheckerboardDetector::DetectFullCheckerboard(const cv::Mat &mat, int width, int height, vector<vector<Vector2dd> > *straights, int precise,
                                                        int maxIterationCount,
                                                        double minAccuracy){
    int             found;
    Size            boardSize(width, height);
    vector<Point2f> pointbuf;
    SYNC_PRINT(("Start ...\n"));
    found = findChessboardCorners(mat, boardSize, pointbuf,
                                  CALIB_CB_ADAPTIVE_THRESH
                                  +  CALIB_CB_FAST_CHECK );
    if(!found){
          SYNC_PRINT(("Failed to detect.\n"));
        return 0;
    }

    for (unsigned i = 0; i < pointbuf.size(); i++)
    {
         SYNC_PRINT(("Point %f %f\n", pointbuf[i].x,pointbuf[i].y));
    }

    SYNC_PRINT(("Finding lines %ix%i...", width, height));
    if(found && precise){
        cornerSubPix ( mat ,  pointbuf ,  Size ( precise ,  precise ),  Size ( - 1 ,  - 1 ),
          TermCriteria ( CV_TERMCRIT_EPS  +  CV_TERMCRIT_ITER ,  maxIterationCount, minAccuracy ));
    }
    fillStraight(pointbuf, width, height, straights);
    return found;
}

OpenCvCheckerboardDetector::BoardAlign OpenCvCheckerboardDetector::DetectPartCheckerboardH(const cv::Mat &mat, const CheckerboardDetectionParameters &params, ObservationList *observationList){
//    int found;
//    vector<Point2f> pointbuf;
//    int widthOfPart = 0;
//    int width = params.hCrossesCount();
//    int height = params.vCrossesCount();
//    for(unsigned i = width; i > 2; i--)
//    {
//        Size boardSize(i, height);
//        SYNC_PRINT(("Try %ix%i",i, height));
//        PreciseTimer timer = PreciseTimer::currentTime();

//        found = findChessboardCorners( mat, boardSize, pointbuf, CV_CALIB_CB_ADAPTIVE_THRESH );
//        if(found)
//        {
////            fillStraight(pointbuf, width, height, straights);
//            widthOfPart = i;
//            double left = pointbuf.at(0).x;
//            double right = pointbuf.at(width - 1).x;
//            for(unsigned j = 1; j < height; j++)
//            {
//                if(left   > pointbuf.at(width * j).x)
//                    left  = pointbuf.at(width * j).x;
//                if(right  > pointbuf.at(width * (j + 1) - 1).x)
//                    right = pointbuf.at(width * (j + 1) - 1).x;
//            }
//            if(left < right)
//            {
//                SYNC_PRINT((" LEFT\n"));
//                return BoardAlign::LEFT;
//            }
//            else
//            {
//                SYNC_PRINT((" RIGHT\n"));
//                return BoardAlign::RIGHT;
//            }
//        }
//        SYNC_PRINT((" failed. Time: %PRIu64 us\n", timer.usecsToNow()));
//    }
    return BoardAlign::NONE;
}

bool OpenCvCheckerboardDetector::fastCheckCheckerboard(const cv::Mat &mat, Size boardSize){
    vector<Point2f> pointbuf;
    return findChessboardCorners( mat, boardSize, pointbuf, CALIB_CB_FAST_CHECK );
}

OpenCvCheckerboardDetector::BoardAlign OpenCvCheckerboardDetector::DetectPartCheckerboardV(const cv::Mat &mat, const CheckerboardDetectionParameters &params, ObservationList *observationList){
    SYNC_PRINT(("Start  !!!!!!!!!!!!!!!!!!!!!!!\n"));

    int width = params.hCrossesCount();
    int height = params.vCrossesCount();
    for(unsigned j = height; j > 2; j--)
    {
        Size boardSize(width, j);
        if(fastCheckCheckerboard(mat,boardSize)){
            SYNC_PRINT(("--------- Fast found %i --------------\n", j));
            vector<Point2f> pointbuf;
            boardSize = Size(width, j);
            if(findChessboardCorners( mat, boardSize, pointbuf, CALIB_CB_FAST_CHECK )){
                SYNC_PRINT(("--------- Found     %i --------------\n", j));

                Mat out(mat);
                DrawCheckerboardIndex(out,pointbuf);
                cv::imwrite("pointIndexes.jpg",out);

                double top = pointbuf.at(0).y;
                double bottom = pointbuf.at(width - 1).y;
                for(unsigned i = 0; i < width-1; i++)
                {
                    if( bottom > pointbuf.at(j * i).y)
                        bottom = pointbuf.at(j * i).y;
                    if( top    > pointbuf.at(j * (i + 1) - 1).y)
                        top    = pointbuf.at(j * (i + 1) - 1).y;
                }

                cornerSubPix ( mat ,  pointbuf ,  Size ( params.preciseDiameter() , params.preciseDiameter() ),  Size ( - 1 ,  - 1 ),
                  TermCriteria ( CV_TERMCRIT_EPS  +  CV_TERMCRIT_ITER , params.iterationCount(), params.minAccuracy() ));

                if(top < bottom)
                {
                    SYNC_PRINT((" TOP\n"));
                    fillPoints(pointbuf, Size(width, height), boardSize, Size(params.cellSize(),params.cellSize()), BoardAlign::TOP, observationList);
                    return BoardAlign::TOP;
                }
                else
                {
                    SYNC_PRINT((" BOTTOM\n"));
                    fillPoints(pointbuf, Size(width, height), boardSize, Size(params.cellSize(),params.cellSize()), BoardAlign::BOTTOM, observationList);
                    return BoardAlign::BOTTOM;
                }
            }
        }

    }
    SYNC_PRINT((" none\n"));
    return BoardAlign::NONE;
}

void OpenCvCheckerboardDetector::fillPoints(const vector<Point2f> &pointbuf, Size fullSize, Size partSize, Size cellSize, BoardAlign alignment, ObservationList *observationList){

    observationList = new ObservationList();

    if(alignment == BoardAlign::BOTTOM){

        int checkerboardShiftX = - fullSize.width;
        int checkerboardShiftY = - partSize.height;

        SYNC_PRINT(("%i %i buff size: %i list size: %i\n",checkerboardShiftX,checkerboardShiftY,pointbuf.size(), observationList->size()));

        for(int i = fullSize.width; i > 0; i--){
            for(int j = partSize.height; j > 0; j--){

                Point2f p(pointbuf[j * i - 1]);
                SYNC_PRINT(("i %i  j %i = %i  %i  = %i  %i \n", i, j,checkerboardShiftX + i,checkerboardShiftY + j, p.x, p.y));

                PointObservation point;
                point.x() = cellSize.width  * (checkerboardShiftX + i);
                point.y() = cellSize.height * (checkerboardShiftY + j);
                point.z() = 0;
                point.u() = p.x;
                point.v() = p.y;

                observationList->push_back(point);
            }
            SYNC_PRINT(("\nnext\n"));
        }
    }else if(alignment == BoardAlign::TOP){
        for(int i = 0; i < partSize.width;i++){
            for(int j = 0; j < fullSize.height;j++){
                PointObservation observation(Vector3dd(cellSize.width * i, cellSize.height * j, 0),Vector2dd(pointbuf.at(j * fullSize.height + i).x,pointbuf.at(j * fullSize.height + i).y));
                observationList->push_back(observation);
            }
        }
    }else if(alignment == BoardAlign::LEFT){
        for(int i = 0; i < fullSize.width;i++){
            for(int j = 0; j < partSize.height;j++){
                PointObservation observation(Vector3dd(cellSize.width * i, cellSize.height * j, 0),Vector2dd(pointbuf.at(j * fullSize.height + i).x,pointbuf.at(j * fullSize.height + i).y));
                observationList->push_back(observation);
            }
        }
    }else if(alignment == BoardAlign::RIGHT){
        for(int i = 0; i < fullSize.width;i++){
            for(int j = 0; j < partSize.height;j++){
                PointObservation observation(Vector3dd(cellSize.width * i, cellSize.height * (fullSize.width - partSize.width + j), 0),Vector2dd(pointbuf.at(j * fullSize.height + i).x,pointbuf.at(j * fullSize.height + i).y));
                observationList->push_back(observation);
            }
        }
    }
}

void OpenCvCheckerboardDetector::fillStraight(const vector<Point2f> &pointbuf, int width, int height, SelectableGeometryFeatures *lineList)
{
    for(int i = 0; i < height; i++)
    {
        SelectableGeometryFeatures::VertexPath * path = lineList->appendNewPath();
        for(int j = 0; j < width; j++)
        {
            Vector2dd point(pointbuf.at(i * width + j).x, pointbuf.at(i * width + j).y);
            lineList->addVertexToPath(lineList->appendNewVertex(point), path);
        }
    }

    for(int j = 0; j < width; j++)
    {
        SelectableGeometryFeatures::VertexPath * path = lineList->appendNewPath();
        for(int i = 0; i < height; i++)
        {
            Vector2dd point(pointbuf.at(i * width + j).x, pointbuf.at(i * width + j).y);
            lineList->addVertexToPath(lineList->appendNewVertex(point), path);
        }
    }
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
    }
}

void OpenCvCheckerboardDetector::fillStraight(const vector<Point2f> &buffer, int width, int height, vector<vector<Vector2dd> > *straights)
{
    SYNC_PRINT(("--------- %i --------------\n", buffer.at(1).x));
    for (unsigned ih = 0; ih < height; ih++)
    {
        vector<Vector2dd> straight;
        for (unsigned iw = 0; iw < width; iw++)
        {
            straight.push_back(Vector2dd(buffer.at(ih * width + iw).x,buffer.at(ih * width + iw).y));
        }
        straights->push_back(straight);
    }
    for (unsigned iw = 0; iw < width; iw++)
    {
        vector<Vector2dd> straight;
        for (unsigned ih = 0; ih < height; ih++)
        {
            straight.push_back(Vector2dd(buffer.at(iw + width * ih).x,buffer.at(iw + width * ih).y));
        }
        straights->push_back(straight);
    }
}
