#include "bufferReaderProvider.h"

BufferReader* BufferReaderProvider::getBufferReader(const std::string &filename) {
	for(auto p: providers) {
		if(p->provides(filename)) {
			return p->getBufferReader(filename);
		}
	}
	assert(false);
	return 0;
}

void BufferReaderProvider::add(BufferReaderProviderImpl *provider) {
	providers.push_back(provider);
}

BufferReaderProvider::~BufferReaderProvider() {
	for(auto p: providers)
		delete p;
	providers.clear();
}

BufferReaderProvider& BufferReaderProvider::getInstance() {
	static BufferReaderProvider provider;
	return provider;
}

BufferReaderProvider::BufferReaderProvider() {
}


