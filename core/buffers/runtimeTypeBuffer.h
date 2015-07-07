#ifndef RUNTIMETYPEBUFFER_H
#define RUNTIMETYPEBUFFER_H

#include <cstring>
#include <string>
#include <iostream>
#include <cassert>
#include <cstdlib>
#include <stdint.h> // for those who use outdated compilers


#define BUFFER_TYPE_TO_STRING(str) \
	case str: \
		return #str;
#define BUFFER_TYPE_FROM_STRING(str) \
	if(!strcmp(name, #str)) \
		type =str;
#define BUFFER_SIZE(str, sz) \
	case str: \
		return sz;
#define BUFFER_CVTYPE(str, type) \
	case str: \
		return type;
#define BUFFER_TYPECV(str, cvtype) \
	case cvtype:\
		type = str;\
		break;
		

#ifndef CV_8U
#define CV_8U 0
#endif

#ifndef CV_32F
#define CV_32F 5
#endif

struct BufferType {
	enum RuntimeBufferDataType {
		U8,
		F32
	};

	int type;

#if 0
	BufferType(RuntimeBufferDataType type = RuntimeBufferDataType::U8) : type(type) {}
#endif

	BufferType(int cvType = 0) {
		switch(cvType) {
			BUFFER_TYPECV(U8, CV_8U);
			BUFFER_TYPECV(F32, CV_32F);
			default:
				type = U8;
		}
	}
	BufferType(const char *name) {
		init(name);
	}
	BufferType(const std::string &name) {
		init(name.c_str());
	}
	operator int() const { return type; }
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
		type = U8;
		BUFFER_TYPE_FROM_STRING(U8);
		BUFFER_TYPE_FROM_STRING(F32);
	}
};


// TODO: add row-aligned allocation & getters
class RuntimeTypeBuffer {
public:
	RuntimeTypeBuffer() : data(0), rows(0), cols(0), sz(0), type(BufferType::U8) {}
	RuntimeTypeBuffer(const size_t &rows, const size_t &cols, const BufferType &type =BufferType::U8) : rows(rows), cols(cols), sz(type.getSize()), type(type) {
		allocate();
	}
	RuntimeTypeBuffer(const size_t &rows, const size_t &cols, const void* data, const BufferType &type = BufferType::U8) : rows(rows), cols(cols), sz(type.getSize()), type(type) {
		allocate();
		copy((uint8_t*)data);
	}

	~RuntimeTypeBuffer() {
		free(data);
	}

	RuntimeTypeBuffer(const RuntimeTypeBuffer &b) : rows(b.rows), cols(b.cols), sz(b.sz), type(b.type) {
		allocate();
		copy(b.data);
	}

// Our compiler is too old for cute move semantics
#if 0
	RuntimeTypeBuffer(RuntimeTypeBuffer &&b) : data(b.data), rows(b.rows), cols(b.cols), sz(b.sz), type(b.type) {
		b.data = 0;
	}
#endif

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

// Our compiler is too old for cute move semantics
#if 0
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
#endif

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
		data = (uint8_t*)malloc(bytes);
	}
	void copy(uint8_t *src) {
		memcpy(data, src, getDataSize());
	}
	uint8_t *data;
	size_t rows;
	size_t cols;
	size_t sz;
	BufferType type;
};

std::ostream& operator<<(std::ostream &os, const RuntimeTypeBuffer &b);
std::istream& operator>>(std::istream &is, RuntimeTypeBuffer &b);

#endif
