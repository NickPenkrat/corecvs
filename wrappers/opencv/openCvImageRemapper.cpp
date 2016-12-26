#include "openCvImageRemapper.h"
#include "opencv2/imgproc.hpp"

void convertRGB(const corecvs::RGB24Buffer& buffer, cv::Mat& mat)
{
	const int height = buffer.getH();
	const int width = buffer.getW();

	mat = cv::Mat(height, width, CV_8UC4);
	for (int j = 0; j < height; j++)
	{
		for (int i = 0; i < width; i++)
			mat.at<corecvs::RGBColor>(i, j) = buffer.element(j, i);
	}
}

void convertRGB(const cv::Mat& mat, corecvs::RGB24Buffer& buffer)
{
	const int height = mat.rows;
	const int width = mat.cols;

	buffer = corecvs::RGB24Buffer(height, width, false);
	for (int j = 0; j < height; j++)
	{
		for (int i = 0; i < width; i++)
			buffer.element(j, i) = mat.at<corecvs::RGBColor>(i, j);
	}
}

void convertMap(corecvs::DisplacementBuffer &transform, cv::Mat& mat)
{
	const int height = transform.getH();
	const int width = transform.getW();

	mat = cv::Mat(height, width, CV_32FC2);
	for (int j = 0; j < height; j++)
	{
		for (int i = 0; i < width; i++)
		{
			corecvs::Vector2dd elementd = transform.element(j, i);
			corecvs::Vector2df elementf(elementd.x(), elementd.y());
			mat.at<corecvs::Vector2df>(i, j) = elementf;
		}	
	}
}

void remap(corecvs::RGB24Buffer &src, corecvs::RGB24Buffer &dst, corecvs::DisplacementBuffer &transform)
{
	cv::Mat input, output;
	cv::Mat map;

	convertRGB(src, input);
	convertMap(transform, map);
	cv::remap(input, output, map, cv::Mat(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
	convertRGB(output, dst);
}
