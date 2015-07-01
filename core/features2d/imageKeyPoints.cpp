#include "imageKeyPoints.h"

#include <cassert>
#include <iomanip>
#include <fstream>


std::ostream& operator<<(std::ostream& os, const KeyPoint& kp) {
	os << std::setprecision(15) <<  std::scientific;

	os << std::setw(20) << kp.x << "\t"
	   << std::setw(20) << kp.y << "\t"
	   << std::setw(20) << kp.size << "\t"
	   << std::setw(20) << kp.angle << "\t"
	   << std::setw(20) << kp.response << "\t"
	   << std::setw(20) << kp.octave << "\t";
	return os;
}

std::istream& operator>>(std::istream& is, KeyPoint& kp) {
	is >> kp.x >> kp.y >> kp.size >> kp.angle >> kp.response >> kp.octave;
	return is;
}

std::ostream& operator<<(std::ostream& os, const ImageKeyPoints& kp) {
	kp.save(os);
	return os;
}

std::istream& operator>>(std::istream& is, ImageKeyPoints& kp) {
	kp.load(is);
	return is;
}


void ImageKeyPoints::load(const std::string &filename) {
	std::ifstream is;
	is.open(filename, std::ifstream::in);
	load(is);
}

void ImageKeyPoints::save(const std::string &filename) const {
	std::ofstream os;
	os.open(filename, std::ofstream::out);
	save(os);
}

void ImageKeyPoints::load(std::istream& is) {
	size_t M;
	is >> M;
	keyPoints.resize(M);

	for(size_t j = 0; j < M; ++j) {
		assert(is);
#if 0
		is  >> keyPoints[j].pt.x 
			>> keyPoints[j].pt.y 
			>> keyPoints[j].size
			>> keyPoints[j].angle
			>> keyPoints[j].response
			>> keyPoints[j].octave;
#else
		is >> keyPoints[j];
#endif
	}
}

void ImageKeyPoints::save(std::ostream& os) const {
	size_t M = keyPoints.size();
	os << M << std::endl;
	os.precision(9);

	for(size_t j = 0; j < M; ++j) {
		assert(os);
#if 0
		os  << std::setw(15) <<  keyPoints[j].pt.x << "\t"
			<< std::setw(15) <<  keyPoints[j].pt.y  << "\t"
			<< std::setw(15) <<  keyPoints[j].size << "\t"
			<< std::setw(15) <<  keyPoints[j].angle << "\t"
			<< std::setw(15) <<  keyPoints[j].response << "\t"
			<< std::setw(15) <<  keyPoints[j].octave << "\t"
			<< std::endl;
#else
		os << keyPoints[j] << std::endl;
#endif
	}
}

std::istream& operator>>(std::istream &is, ImageDescriptors &b) {
	std::string desc_type;
	is >> desc_type;
	b.type = desc_type;
	is >> b.mat;
	return is;
}

std::ostream& operator<<(std::ostream &os, const ImageDescriptors &b) {
	std::string desc_type = (std::string)b.type;
	os << desc_type << std::endl;
	os << b.mat;
	return os;
}

Image::Image(const size_t &id) : id(id) {
}

Image::Image(const size_t &id, const std::string& filename) : id(id), filename(filename) {
}

void DescriptorBuffer::load(std::istream &is) {
	is >> (*this);
}

void DescriptorBuffer::save(std::ostream &os) const {
	os << (*this);
}

void ImageDescriptors::load(std::istream &is) {
	is >> (*this);
}

void ImageDescriptors::save(std::ostream &os) const {
	os << (*this);
}

void ImageDescriptors::load(const std::string &filename) {
	std::ifstream is;
	is.open(filename, std::istream::in);
	assert(is);

	load(is);
}

void ImageDescriptors::save(const std::string &filename) const {
	std::ofstream os;
	os.open(filename, std::ostream::out);
	assert(os);

	save(os);
}

std::istream& operator>>(std::istream &is, DescriptorBuffer &b) {
	size_t R, C;
	std::string type;
	is >> R >> C >> type;
	b = DescriptorBuffer(R, C, BufferType(type));
	switch(b.type) {
		case BufferDataType::U8:
			for(size_t i = 0; i < R; ++i)
				for(size_t j = 0; j < C; ++j) {
					int v;
					is >> v;
					b.at<uint8_t>(i, j) = v;
				}
			break;
		case BufferDataType::F32:
			for(size_t i = 0; i < R; ++i)
				for(size_t j = 0; j < C; ++j) {
					is >> b.at<float>(i, j);
				}
			break;
	}
	return is;
}

std::ostream& operator<<(std::ostream &os, const DescriptorBuffer &b) {
	size_t R = b.rows, C = b.cols;
	std::string type = (std::string)b.type;

	os << R << " " << C << " " << type << std::endl;
	switch(b.type) {
		case BufferDataType::U8:
			for(size_t i = 0; i < R; ++i) {
				for(size_t j = 0; j < C; ++j) {
					int v = b.at<uint8_t>(i, j);
					os << std::setw(5) << v;
				}
				os << std::endl;
			}
			break;
		case BufferDataType::F32:
			os << std::setprecision(15) << std::scientific;
			for(size_t i = 0; i < R; ++i) {
				for(size_t j = 0; j < C; ++j) {
					os << std::setw(20) << b.at<float>(i, j);
				}
				os << std::endl;
			}
			break;
	}
	return os;
}
