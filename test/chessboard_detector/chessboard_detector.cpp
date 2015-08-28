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
#include "circlePatternGenerator.h"

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
    cv::imwrite(filename, im);
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

void writeImage(const std::string &old, const std::string &nw, corecvs::RGB24Buffer &img)
{
    cv::Mat im = cv::imread(old);
    for (int i = 0; i < im.rows; ++i)
    {
        for (int j = 0; j < im.cols; ++j)
        {
            im.at<uint8_t>(i, j * 3 + 2) = img.element(i, j).r();
            im.at<uint8_t>(i, j * 3 + 1) = img.element(i, j).g();
            im.at<uint8_t>(i, j * 3 + 0) = img.element(i, j).b();
        }
    }
    cv::imwrite(nw, im);
}

int main(int argc, char **argv)
{
    CirclePatternGenerator gen(256);

    gen.addToken(0, 0.08, {
        corecvs::Vector2dd(0.6018, 0.7052)});

    gen.addToken(0, 0.08, {
        corecvs::Vector2dd(0.1801, 0.8213)});
    gen.addToken(0, 0.08, {
        corecvs::Vector2dd(0.8090, 0.1844),
        corecvs::Vector2dd(0.3729, 0.4622)});
    gen.addToken(0, 0.08, {
        corecvs::Vector2dd(0.2631, 0.5647),
        corecvs::Vector2dd(0.5584, 0.1996)});
    gen.addToken(0, 0.08, {
        corecvs::Vector2dd(0.2905, 0.3433),
        corecvs::Vector2dd(0.1678, 0.6466)});
    gen.addToken(0, 0.08, {
        corecvs::Vector2dd(0.5134, 0.8329),
        corecvs::Vector2dd(0.4391, 0.4329),
        corecvs::Vector2dd(0.6632, 0.2611)});
    gen.addToken(0, 0.08, {
        corecvs::Vector2dd(0.6848, 0.5636),
        corecvs::Vector2dd(0.7078, 0.1697),
        corecvs::Vector2dd(0.1735, 0.7444)});
    gen.addToken(0, 0.08, {
        corecvs::Vector2dd(0.6951, 0.7159),
        corecvs::Vector2dd(0.4921, 0.4379),
        corecvs::Vector2dd(0.1801, 0.4451)});
#if 0
    gen.addToken(0, 1.0/10.0, {corecvs::Vector2dd(0.3278, 0.6332)});
    gen.addToken(0, 1.0/10.0, {corecvs::Vector2dd(0.3354, 0.2011)});
    gen.addToken(0, 1.0/10.0, {corecvs::Vector2dd(0.2061, 0.4158), corecvs::Vector2dd(0.4273, 0.6529)});
    gen.addToken(0, 1.0/10.0, {corecvs::Vector2dd(0.4998, 0.2036), corecvs::Vector2dd(0.2283, 0.5174)});
#endif
#if 0
    std::cout << "stride = " << gen.patterns[0][0].stride << "; " << "width = " << gen.patterns[0][0].w << ";" << std::endl;
    std::cout << "pattern = [";
    for (auto &v: gen.patterns)
    {
        for (auto &p: v)
        {
            int w = p.w;
            for (int y = 0; y < w; ++y)
            {
                for (int x = 0; x < w; ++x)
                {
                    std::cout << p.element(y, x) << " ";
                }
                std::cout <<  "; ";
            }
        }
    }
    std::cout << "];" << std::endl;
#endif
#if 1
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
    ChessBoardDetectorParams params;
    params.w = 10;
    params.h = 7;
    params.mode = ChessBoardDetectorMode::BEST;
    ChessboardDetector detector(params);
    detector.detectPattern(img);

    corecvs::ObservationList observations;
    detector.getPointData(observations);

    DpImage img2;
    readImage(filename, img2);
    detector.classify(img2, gen, img);
    writeImage(filename, "output.png", img);

    std::cout << "board = [";
    for (auto o: observations)
    {
        std::cout << o.projection[0] << " " << o.projection[1] << ";";
    }
    std::cout << "];" << std::endl;
    return 0;
#endif
}
