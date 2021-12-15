#include "resizeApplicator.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

ResizeApplicator::ResizeApplicator()
{

}

bool ResizeApplicator::applySpecialParameters(const QStringList &params)
{
    scale = 0.5;
    if(params.size() > 4)
    {
        bool ready;
        scale = params.at(4).toDouble(&ready);
        if(!ready)
        {
            std::cerr << "Incorrect scale value (not a number), setting to 1\n";
            scale = 0.5;
            ready = true;
        }
    }

    return true;
}

bool ResizeApplicator::applyAugment(const std::string &inputFile, const std::string &outputFile)
{
    auto inputImage = cv::imread(inputFile, cv::IMREAD_COLOR);
    cv::Mat outputImage;
    if (inputImage.data == nullptr)
        return false;

    cv::resize(inputImage, outputImage, cv::Size(0,0),scale, scale, cv::INTER_LINEAR);
    return cv::imwrite(outputFile, outputImage);
}

std::string ResizeApplicator::helpMessage()
{
    return "Usage: resize from_template to_path [scale=1.0]";
}
