#include "global.h"

#include <vector>
#include <cassert>
#include <iostream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "chessBoardDetector.h"
#include "chessBoardCornerDetector.h"
#include "chessBoardAssembler.h"

void usage()
{
    std::cout << "test_chessboard_detector image [W H]\n"
        "  * image - input image\n"
        "  * W x H - expected full size of pattern\n" << std::endl;

}

bool checkIfExists(const char *filename)
{
    std::ifstream ifs;
    ifs.open(filename, std::ios_base::in);

    return (bool)ifs;
}


bool parseArgs(int argc, char **argv, std::string &filename, int &W, int &H)
{
    if (argc < 2 || argc > 2 && argc < 4 || argc > 4)
        return false;
    filename = std::string(argv[1]);
    if (!checkIfExists(argv[1]))
    {
        std::cout << "File " << filename << " not found!" << std::endl;
        return false;
    }
    if (argc > 2)
    {
        W = std::stoi(argv[2]);
        H = std::stoi(argv[3]);
    }
    return true;
}


void readImage(const std::string &filename, DpImage &img)
{
    cv::Mat im = cv::imread(filename);
    im.convertTo(im, CV_64FC1, 1.0 / 255.0);
    // XXX:  OPENCV DOES NOT SUPPORTS FP64 RGB->GRAY 
    // TODO: BTW, I hope that it is always BGR order, is it correct?!
    img = DpImage(im.rows, im.cols);
    for (int i = 0; i < im.rows; ++i)
    {
        for (int j = 0; j < im.cols; ++j)
        {
                (img.element(i, j) = 0.299 * im.at<double>(i, j * 3 + 2) + 0.587 * im.at<double>(i, j * 3 + 1) + 0.114 * im.at<double>(i, j * 3))
                ;
        }
    }
}

void readImage(const std::string &filename, corecvs::RGB24Buffer &img)
{
    cv::Mat im = cv::imread(filename);
    im.convertTo(im, CV_64FC1, 1.0);
    img = corecvs::RGB24Buffer(im.rows, im.cols);
    for (int i = 0; i < im.rows; ++i)
    {
        for (int j = 0; j < im.cols; ++j)
        {
            img.element(i, j) = corecvs::RGBColor(im.at<double>(i, j * 3 + 2), im.at<double>(i, j * 3 + 1), im.at<double>(i, j * 3));
        }
    }
}

int main(int argc, char **argv)
{
    int W, H;
    std::string filename;
    if(!parseArgs(argc, argv, filename, W, H))
    {
        usage();
        return 0;
    }
#if 0
    DpImage img;
#else
    corecvs::RGB24Buffer img;
#endif

    readImage(filename , img);
    CheckerboardDetectionParameters params;
    BoardAlignerParams alignerParams = BoardAlignerParams::GetOldBoard();
    ChessboardDetector detector(params, alignerParams);
    detector.detectPattern(img);

    corecvs::ObservationList observations;
    detector.getPointData(observations);

    std::cout << "board = [";
    for (auto o: observations)
    {
        std::cout << o.projection[0] << " " << o.projection[1] << ";";
    }
    std::cout << "];" << std::endl;
    return 0;
}
