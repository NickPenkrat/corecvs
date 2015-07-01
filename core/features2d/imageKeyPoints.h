#ifndef IMAGEKEYPOINTS_H
#define IMAGEKEYPOINTS_H

#include <vector>
#include <iostream>
#include <cstdint>
#include <cstdlib>
#include <cassert>
#include <cstring>


typedef std::string DescriptorType;
typedef std::string DetectorType;

struct KeyPoint {
	KeyPoint(double x = 0.0, double y = 0.0, double size = 0.0, double angle = -1.0, double response = 0.0, int octave = 0)
		: x(x), y(y), size(size), angle(angle), response(response), octave(octave) {
	}

#if 0
	KeyPoint(const cv::KeyPoint &kp)
		: x(kp.pt.x), y(kp.pt.y), size(kp.size), angle(kp.angle), response(kp.response), octave(kp.octave) {
	}

	operator cv::KeyPoint() const {
		return cv::KeyPoint(x, y, size, angle, response, octave);
	}
#endif

	friend std::ostream& operator<<(std::ostream& os, const KeyPoint &kp);
	friend std::istream& operator>>(std::istream& is, KeyPoint &kp);

	double x;
	double y;
	double size;
	double angle;
	double response;
	int octave;
};

struct ImageKeyPoints {
	void save(const std::string &filename) const;
	void load(const std::string &filename);
	void save(std::ostream &os) const;
	void load(std::istream &is);

	friend std::ostream& operator<<(std::ostream& os, const ImageKeyPoints& kp);
	friend std::istream& operator>>(std::istream& is, ImageKeyPoints& kp);
	
	std::vector<KeyPoint> keyPoints;
};

enum class BufferDataType {
	U8,
	F32
};

#define BUFFER_TYPE_TO_STRING(str) \
	case BufferDataType::str: \
		return #str;
#define BUFFER_TYPE_FROM_STRING(str) \
	if(!strcmp(name, #str)) \
		type = BufferDataType::str;
#define BUFFER_SIZE(str, sz) \
	case BufferDataType::str: \
		return sz;
#define BUFFER_CVTYPE(str, type) \
	case BufferDataType::str: \
		return type;
#define BUFFER_TYPECV(str, cvtype) \
	case cvtype:\
		type = BufferDataType::str;\
		break;
		

#ifndef CV_8U
#define CV_8U 0
#endif

#ifndef CV_32F
#define CV_32F 5
#endif

struct BufferType {
	BufferDataType type;

	BufferType(BufferDataType type = BufferDataType::U8) : type(type) {}
	explicit BufferType(size_t cvType) {
		switch(cvType) {
			BUFFER_TYPECV(U8, CV_8U);
			BUFFER_TYPECV(F32, CV_32F);
			default:
				type = BufferDataType::U8;
		}
#if 0
		std::cout << "CVTYPE: " << cvType << "( F: " << CV_32F << ")" << std::endl;
#endif
	}
	BufferType(const char *name) {
		init(name);
	}
	BufferType(const std::string &name) {
		init(name.c_str());
	}
	operator BufferDataType() const { return type; }
	explicit operator const char*() const {
		switch(type) {
			BUFFER_TYPE_TO_STRING(U8);
			BUFFER_TYPE_TO_STRING(F32);
		}
		assert(false);
		return "INVALID";
	}
	explicit operator std::string() const {
		return std::string((const char*)(*this));
	}
	size_t getSize() const {
		switch(type) {
			BUFFER_SIZE(U8, sizeof(uint8_t));
			BUFFER_SIZE(F32, sizeof(float));
		}
		assert(false);
		return 0;
	}
	size_t getCvType() const {
		switch(type) {
			BUFFER_CVTYPE(U8, CV_8U);
			BUFFER_CVTYPE(F32, CV_32F);
		}
		assert(false);
		return 0;
	}
private:
	void init(const char* name) {
		type = BufferDataType::U8;
		BUFFER_TYPE_FROM_STRING(U8);
		BUFFER_TYPE_FROM_STRING(F32);
	}
};


// TODO: add row-aligned allocation & getters
class DescriptorBuffer {
public:
	DescriptorBuffer() : data(0), rows(0), cols(0), sz(0), type(BufferDataType::U8) {}
	DescriptorBuffer(const size_t &rows, const size_t &cols, const BufferType &type = BufferDataType::U8) : rows(rows), cols(cols), sz(type.getSize()), type(type) {
		allocate();
	}
	DescriptorBuffer(const size_t &rows, const size_t &cols, const void* data, const BufferType &type = BufferDataType::U8) : rows(rows), cols(cols), sz(type.getSize()), type(type) {
		allocate();
		copy((uint8_t*)data);
	}

#if 0
	DescriptorBuffer(const cv::Mat &mat) : rows(mat.rows), cols(mat.cols) {
		switch(mat.type()) {
			case CV_8U:
				type = BufferDataType::U8;
				break;
			case CV_32F:
				type = BufferDataType::F32;
				break;
			default:
				assert(false);
		}
		sz = type.getSize();
		allocate();
		copy((uint8_t*)mat.data);
	}
#endif

	~DescriptorBuffer() {
		free(data);
	}

	DescriptorBuffer(const DescriptorBuffer &b) : rows(b.rows), cols(b.cols), sz(b.sz), type(b.type) {
		allocate();
		copy(b.data);
	}

	DescriptorBuffer(DescriptorBuffer &&b) : data(b.data), rows(b.rows), cols(b.cols), sz(b.sz), type(b.type) {
		b.data = 0;
	}

	DescriptorBuffer& operator=(const DescriptorBuffer &b) {
		if(this == &b)
			return *this;
		free(data);
		rows = b.rows;
		cols = b.cols;
		sz = b.sz;
		type = b.type;

		allocate();
		copy(b.data);
		return *this;
	}

	DescriptorBuffer& operator=(DescriptorBuffer &&b) {
		if(this == &b)
			return *this;

		free(data);
		rows = b.rows;
		cols = b.cols;
		sz = b.sz;
		type = b.type;
		data = b.data;
		b.data = 0;
		return *this;
	}

	bool isValid() const {
		return data != 0;
	}

	size_t getRowSize() const {
		return sz * cols;
	}

	size_t getElSize() const {
		return sz;
	}

	size_t getDataSize() const {
		return rows * getRowSize();
	}

#if 0
	operator cv::Mat() const { 
		// Note it is not const really! flawed(?) OpenCV design does not allow us to have const data
		return cv::Mat(rows, cols, type.getCvType(), (void*)data);
	}
#endif

	template<typename T>
	T& at(const size_t &i, const size_t &j) {
		return *(T*)(data + sz * (i * cols + j));
	}
	template<typename T>
	const T& at(const size_t &i, const size_t &j) const {
		return *(T*)(data + sz * (i * cols + j));
	}
	template<typename T>
	T* row(const size_t &i) {
		return (T*)(data + sz * (i * cols ));
	}
	// FIXME
	template<typename T>
	T* row(const size_t &i) const {
		return (T*)(data + sz * (i * cols));
	}
	inline size_t getRows() const {
		return rows;
	}
	inline size_t getCols() const {
		return cols;
	}
	BufferType getType() const {
		return type;
	}

	void load(std::istream &is);
	void save(std::ostream &os) const;
	void load(const std::string &filename);
	void save(const std::string &filename) const;
	friend std::ostream& operator<<(std::ostream &os, const DescriptorBuffer &b);
	friend std::istream& operator>>(std::istream &is, DescriptorBuffer &b);
private:
	void allocate() {
		size_t bytes = getDataSize();
		data = (uint8_t*)aligned_alloc(16, bytes);
	}
	void copy(uint8_t *src) {
		memcpy(data, src, getDataSize());
	}
#if 0
	size_t getDataSize() const {
		return sz * cols * rows;
	}
#endif
	uint8_t *data;
	size_t rows;
	size_t cols;
	size_t sz;
	BufferType type;
};

struct ImageDescriptors {
	DescriptorBuffer mat;
	DescriptorType type;

	void load(std::istream &is);
	void save(std::ostream &os) const;
	void load(const std::string &filename);
	void save(const std::string &filename) const;
	friend std::ostream& operator<<(std::ostream &os, const ImageDescriptors &b);
	friend std::istream& operator>>(std::istream &is, ImageDescriptors &b);
};

struct Image {
	ImageDescriptors descriptors;
	ImageKeyPoints keyPoints;
	
	size_t id;
	std::string filename;


	Image(const size_t &id);
	Image(const size_t &id, const std::string &filename);
};

#endif
