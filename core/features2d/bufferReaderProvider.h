#ifndef BUFFERREADERPROVIDER_H
#define BUFFERREADERPROVIDER_H

#include <cassert>
#include <string>
#include <vector>

#include "runtimeTypeBuffer.h"

class BufferReader {
public:
	virtual RuntimeTypeBuffer read(const std::string &s) = 0;
	virtual ~BufferReader() {}
};

class BufferReaderProviderImpl {
	public:
		virtual BufferReader* getBufferReader(const std::string &filename) = 0;
		virtual bool provides(const std::string &filename) = 0;
		virtual ~BufferReaderProviderImpl() {}
	protected:

};

class BufferReaderProvider {
public:
	void add(BufferReaderProviderImpl *provider);
	BufferReader* getBufferReader(const std::string &filename);
	static BufferReaderProvider& getInstance();
	~BufferReaderProvider();
private:
	BufferReaderProvider();
	BufferReaderProvider(const BufferReaderProvider&);
	BufferReaderProvider& operator=(const BufferReaderProvider&);
	std::vector<BufferReaderProviderImpl*> providers;
};


#endif
