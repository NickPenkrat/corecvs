#include "blurApplicator.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

BlurApplicator::BlurApplicator()
{

}

bool BlurApplicator::applySpecialParameters(const QStringList &params)
{
    kernelSize = 3;
    if(params.size() > 4)
    {
        bool ready;
        kernelSize = params.at(4).toInt(&ready);
        if(!ready)
        {
            std::cerr << "Incorrect kernel size (not an integer), setting to 3\n";
            kernelSize = 3;
            ready = true;
        }
        else
        {
            if(kernelSize % 2 == 0)
            {
                kernelSize--;
                std::cerr << "Incorrect kernel size(even integer), setting to " << kernelSize << std::endl;
            }
        }
    }

    return true;
}

bool BlurApplicator::applyAugment(const std::string &inputFile, const std::string &outputFile)
{
    auto inputImage = cv::imread(inputFile, cv::IMREAD_COLOR);
    cv::Mat outputImage;
    if (inputImage.data == nullptr)
        return false;

    cv::GaussianBlur(inputImage, outputImage, cv::Size(kernelSize, kernelSize), 0);
    return cv::imwrite(outputFile, outputImage);
}

std::string BlurApplicator::helpMessage()
{
    return "Usage: blur from_template to_path [kernel_size=3]";
}
