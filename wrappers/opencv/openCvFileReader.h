#ifndef OPENCVFILEREADER_H
#define OPENCVFILEREADER_H

#include "bufferReaderProvider.h"


class OpenCvBufferReader : public BufferReader {
public:
	RuntimeTypeBuffer read(const std::string &s);
	~OpenCvBufferReader() {}
};

class OpenCvBufferReaderProvider : public BufferReaderProviderImpl {
public:
	BufferReader* getBufferReader(const std::string &filename);
	bool provides(const std::string &filename);
	~OpenCvBufferReaderProvider() {}
};

void __attribute__ ((constructor)) __attribute__ ((used)) init_opencv_reader_provider(); 

#endif
