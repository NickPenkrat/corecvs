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

#include "qtFileLoader.h"

#include "jsonSetter.h"

void usage()
{
    std::cout << "test_chessboard_detector image\n"
        "  * image - input image\n"
         << std::endl;

}

bool checkIfExists(const char *filename)
{
    std::ifstream ifs;
    ifs.open(filename, std::ios_base::in);

    return (bool)ifs;
}


bool parseArgs(int argc, char **argv, std::string &filename, int &W, int &H)
{
    if (argc < 2 || argc > 2)
        return false;
    filename = std::string(argv[1]);
    if (!checkIfExists(argv[1]))
    {
        std::cout << "File " << filename << " not found!" << std::endl;
        return false;
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
    params.setCellSizeHor (26);
    params.setCellSizeVert(18);
    params.setFitWidth (false);
    params.setFitHeight(false);
    BoardAlignerParams paramsA;
    paramsA.idealWidth = 26;
    paramsA.idealHeight = 18;
    paramsA.type = AlignmentType::FIT_MARKERS;
    paramsA.circleCenters =
    {        
        {
            corecvs::Vector2dd(0.823379, 0.421373),
            corecvs::Vector2dd(0.441718, 0.370170),
            corecvs::Vector2dd(0.269471, 0.166728)
        },
        {
            corecvs::Vector2dd(0.589928, 0.166732),
            corecvs::Vector2dd(0.269854, 0.833333),
            corecvs::Vector2dd(0.692244, 0.589654)
        },
        {
            corecvs::Vector2dd(0.585422, 0.680195),
            corecvs::Vector2dd(0.287252, 0.833178),
            corecvs::Vector2dd(0.166910, 0.441706)
        },
        {
            corecvs::Vector2dd(0.166911, 0.285371),
            corecvs::Vector2dd(0.421789, 0.675665),
            corecvs::Vector2dd(0.579427, 0.174676)
        },
        {
            corecvs::Vector2dd(0.705211, 0.425448),
            corecvs::Vector2dd(0.167029, 0.781583),
            corecvs::Vector2dd(0.170480, 0.403242)
        },
        {
            corecvs::Vector2dd(0.166779, 0.582670),
            corecvs::Vector2dd(0.422544, 0.689812),
            corecvs::Vector2dd(0.733317, 0.828926)
        },
        {
            corecvs::Vector2dd(0.167587, 0.433621),
            corecvs::Vector2dd(0.686839, 0.425617),
            corecvs::Vector2dd(0.833333, 0.746159)
        },
        {
            corecvs::Vector2dd(0.751807, 0.833313),
            corecvs::Vector2dd(0.441399, 0.687871),
            corecvs::Vector2dd(0.596547, 0.166670)
        }
    };
    paramsA.markerCells = 
    {
        std::array<std::pair<int,int>,4>(
            {
                std::make_pair( 3,  1),
                std::make_pair( 4,  1), 
                std::make_pair( 3,  2), 
                std::make_pair( 4,  2)
            }),
        std::array<std::pair<int,int>,4>(

            {
                std::make_pair( 2,  8),
                std::make_pair( 3,  8), 
                std::make_pair( 2,  9), 
                std::make_pair( 3,  9)
            }),
        std::array<std::pair<int,int>,4>(

            {
                std::make_pair(12,  2),
                std::make_pair(13,  2), 
                std::make_pair(12,  3), 
                std::make_pair(13,  3)
            }),
        std::array<std::pair<int,int>,4>(

            {
                std::make_pair(21,  1),
                std::make_pair(22,  1), 
                std::make_pair(21,  2), 
                std::make_pair(22,  2)
            }),
        std::array<std::pair<int,int>,4>(

            {
                std::make_pair(22,  8),
                std::make_pair(23,  8), 
                std::make_pair(22,  9), 
                std::make_pair(23,  9)
            }),
        std::array<std::pair<int,int>,4>(

            {
                std::make_pair(21, 15),
                std::make_pair(22, 15), 
                std::make_pair(21, 16), 
                std::make_pair(22, 16)
            }),
        std::array<std::pair<int,int>,4>(

            {
                std::make_pair(12, 14),
                std::make_pair(13, 14), 
                std::make_pair(12, 15), 
                std::make_pair(13, 15)
            }),
        std::array<std::pair<int,int>,4>(

            {
                std::make_pair( 3, 15),
                std::make_pair( 4, 15), 
                std::make_pair( 3, 16), 
                std::make_pair( 4, 16)
            })
    };
    ChessboardDetector detector(params, paramsA);
    detector.detectPattern(img);

    corecvs::ObservationList observations;
    detector.getPointData(observations);
    detector.drawClassifier(img);

    JSONSetter setter("out.json");
    setter.visit(observations, "observations");

    QTFileLoader().save(std::string("classifier_output.jpg"), &img, 100);

    std::cout << "board = [";
    for (auto o: observations)
    {
        std::cout << o.projection[0] << " " << o.projection[1] << ";";
    }
    std::cout << "];" << std::endl;
    return 0;
}
