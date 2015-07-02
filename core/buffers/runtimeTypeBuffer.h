#ifndef RUNTIMETYPEBUFFER_H
#define RUNTIMETYPEBUFFER_H

#include <cstring>
#include <string>
#include <iostream>
#include <cassert>

enum class RuntimeBufferDataType {
	U8,
	F32
};

#define BUFFER_TYPE_TO_STRING(str) \
	case RuntimeBufferDataType::str: \
		return #str;
#define BUFFER_TYPE_FROM_STRING(str) \
	if(!strcmp(name, #str)) \
		type = RuntimeBufferDataType::str;
#define BUFFER_SIZE(str, sz) \
	case RuntimeBufferDataType::str: \
		return sz;
#define BUFFER_CVTYPE(str, type) \
	case RuntimeBufferDataType::str: \
		return type;
#define BUFFER_TYPECV(str, cvtype) \
	case cvtype:\
		type = RuntimeBufferDataType::str;\
		break;
		

#ifndef CV_8U
#define CV_8U 0
#endif

#ifndef CV_32F
#define CV_32F 5
#endif

struct BufferType {
	RuntimeBufferDataType type;

	BufferType(RuntimeBufferDataType type = RuntimeBufferDataType::U8) : type(type) {}
	explicit BufferType(size_t cvType) {
		switch(cvType) {
			BUFFER_TYPECV(U8, CV_8U);
			BUFFER_TYPECV(F32, CV_32F);
			default:
				type = RuntimeBufferDataType::U8;
		}
	}
	BufferType(const char *name) {
		init(name);
	}
	BufferType(const std::string &name) {
		init(name.c_str());
	}
	operator RuntimeBufferDataType() const { return type; }
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
		type = RuntimeBufferDataType::U8;
		BUFFER_TYPE_FROM_STRING(U8);
		BUFFER_TYPE_FROM_STRING(F32);
	}
};


// TODO: add row-aligned allocation & getters
class RuntimeTypeBuffer {
public:
	RuntimeTypeBuffer() : data(0), rows(0), cols(0), sz(0), type(RuntimeBufferDataType::U8) {}
	RuntimeTypeBuffer(const size_t &rows, const size_t &cols, const BufferType &type = RuntimeBufferDataType::U8) : rows(rows), cols(cols), sz(type.getSize()), type(type) {
		allocate();
	}
	RuntimeTypeBuffer(const size_t &rows, const size_t &cols, const void* data, const BufferType &type = RuntimeBufferDataType::U8) : rows(rows), cols(cols), sz(type.getSize()), type(type) {
		allocate();
		copy((uint8_t*)data);
	}

#if 0
	RuntimeTypeBuffer(const cv::Mat &mat) : rows(mat.rows), cols(mat.cols) {
		switch(mat.type()) {
			case CV_8U:
				type = RuntimeBufferDataType::U8;
				break;
			case CV_32F:
				type = RuntimeBufferDataType::F32;
				break;
			default:
				assert(false);
		}
		sz = type.getSize();
		allocate();
		copy((uint8_t*)mat.data);
	}
#endif

	~RuntimeTypeBuffer() {
		free(data);
	}

	RuntimeTypeBuffer(const RuntimeTypeBuffer &b) : rows(b.rows), cols(b.cols), sz(b.sz), type(b.type) {
		allocate();
		copy(b.data);
	}

	RuntimeTypeBuffer(RuntimeTypeBuffer &&b) : data(b.data), rows(b.rows), cols(b.cols), sz(b.sz), type(b.type) {
		b.data = 0;
	}

	RuntimeTypeBuffer& operator=(const RuntimeTypeBuffer &b) {
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

	RuntimeTypeBuffer& operator=(RuntimeTypeBuffer &&b) {
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
	friend std::ostream& operator<<(std::ostream &os, const RuntimeTypeBuffer &b);
	friend std::istream& operator>>(std::istream &is, RuntimeTypeBuffer &b);
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

std::ostream& operator<<(std::ostream &os, const RuntimeTypeBuffer &b);
std::istream& operator>>(std::istream &is, RuntimeTypeBuffer &b);

#endif
