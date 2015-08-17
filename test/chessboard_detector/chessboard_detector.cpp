#include "global.h"

#include <vector>
#include <cassert>
#include <iostream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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
    // BLAH STUPID OPENCV DOES NOT SUPPORTS FP64 RGB->GRAY 
    // BTW, I hope that it is always BGR order, is it correct?!
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

int main(int argc, char **argv)
{
    int W, H;
    std::string filename;
    if(!parseArgs(argc, argv, filename, W, H))
    {
        usage();
        return 0;
    }
    DpImage img, du, dv, phi, w, c;

    readImage(filename , img);

    std::vector<OrientedCorner> orientedCorners;
    ChessboardCornerDetector detector;
    std::cout << "Detecting corners... " << std::endl;
    detector.detectCorners(img, orientedCorners);
    ChessBoardAssembler assembler;
    std::cout << orientedCorners.size() << " corners detected" << std::endl;
    std::vector<std::vector<std::vector<corecvs::Vector2dd>>> hypothesis;
    std::cout << "Detecting boards... " << std::endl;
    assembler.assembleBoards(orientedCorners, hypothesis);

    for (auto& b: hypothesis)
    {
        for (auto& r: b)
        {
            for (auto& c: r)
            {
                std::cout << c << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl << std::endl;
    }
    return 0;
}
