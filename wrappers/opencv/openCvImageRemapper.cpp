#include "opencv2/imgproc.hpp"
#include "openCvImageRemapper.h"

static void convertRGB(const corecvs::RGB24Buffer& buffer, cv::Mat& mat)
{
	const int height = buffer.getH();
	const int width = buffer.getW();

	mat = cv::Mat(height, width, CV_8UC4);
	for (int j = 0; j < height; j++)
	{
		for (int i = 0; i < width; i++)
			*mat.ptr<uint>(j, i) = buffer.element(j, i).color();		
	}
}

static void convertRGB(const cv::Mat& mat, corecvs::RGB24Buffer& buffer)
{
	const int height = mat.rows;
	const int width = mat.cols;

	buffer = corecvs::RGB24Buffer(height, width, false);
	for (int j = 0; j < height; j++)
	{
		for (int i = 0; i < width; i++)
			buffer.element(j, i).color() = *mat.ptr<uint>(j, i);
	}
}

static void convertMap(corecvs::DisplacementBuffer &transform, cv::Mat& mat)
{
	const int height = transform.getH();
	const int width = transform.getW();

	float minimumX = 100.0f;
	float minimumY = 100.0f;

	float maximumX = -100.0f;
	float maximumY = -100.0f;

	mat = cv::Mat(height, width, CV_32FC2);
	for (int j = 0; j < height; j++)
	{
		for (int i = 0; i < width; i++)
		{
			corecvs::Vector2dd elementd = transform.map(j, i);
			float x = elementd.x();
			minimumX = std::min(x, minimumX);
			maximumX = std::max(x, maximumX);
			float y = elementd.y();
			minimumY = std::min(y, minimumY);
			maximumY = std::max(y, maximumY);
			corecvs::Vector2df elementf(x, y);
			*mat.ptr<corecvs::Vector2df>(j, i) = elementf;
		}	
	}

	cout << endl << "LIMITS" << endl;
	cout << minimumX << endl;
	cout << minimumY << endl;
	cout << maximumX << endl;
	cout << maximumY << endl;
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
