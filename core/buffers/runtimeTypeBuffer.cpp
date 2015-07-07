#include "runtimeTypeBuffer.h"

#include <iomanip>

void RuntimeTypeBuffer::load(std::istream &is) {
	is >> (*this);
}

void RuntimeTypeBuffer::save(std::ostream &os) const {
	os << (*this);
}

std::istream& operator>>(std::istream &is, RuntimeTypeBuffer &b) {
	size_t R, C;
	std::string type;
	is >> R >> C >> type;
	b = RuntimeTypeBuffer(R, C, BufferType(type));
	switch(b.type) {
		case RuntimeBufferDataType::U8:
			for(size_t i = 0; i < R; ++i)
				for(size_t j = 0; j < C; ++j) {
					int v;
					is >> v;
					b.at<uint8_t>(i, j) = v;
				}
			break;
		case RuntimeBufferDataType::F32:
			for(size_t i = 0; i < R; ++i)
				for(size_t j = 0; j < C; ++j) {
					is >> b.at<float>(i, j);
				}
			break;
	}
	return is;
}

std::ostream& operator<<(std::ostream &os, const RuntimeTypeBuffer &b) {
	size_t R = b.rows, C = b.cols;
	std::string type = (std::string)b.type;

	os << R << " " << C << " " << type << std::endl;
	switch(b.type) {
		case RuntimeBufferDataType::U8:
			for(size_t i = 0; i < R; ++i) {
				for(size_t j = 0; j < C; ++j) {
					int v = b.at<uint8_t>(i, j);
					os << std::setw(5) << v;
				}
				os << std::endl;
			}
			break;
		case RuntimeBufferDataType::F32:
			os << std::setprecision(15) << std::scientific;
			for(size_t i = 0; i < R; ++i) {
				for(size_t j = 0; j < C; ++j) {
					os << std::setw(20) << b.at<float>(i, j) << "\t";
				}
				os << std::endl;
			}
			break;
	}
	return os;
}

