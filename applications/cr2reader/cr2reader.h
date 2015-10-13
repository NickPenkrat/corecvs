#pragma once
/*
Converts .cr2 to .ppm or bayer non-filtered .ppm using LibRaw.
LibRaw should be modified to have its methods public
*/
#include "g12Buffer.h"

using corecvs::G12Buffer;

class LibRaw;

class CR2Reader
{
public:
	CR2Reader();
	CR2Reader(const char* filename);

	// does not return boolean success; instead, it returns error code
	int open(const char* filename);
	~CR2Reader();
	int processDCRaw();

	G12Buffer *getBayer(bool shifted = true);
	G12Buffer **getChannels();
	double *getGamm();

	void setBPP(uint bits);
	void setQuality(uint quality);

	int writePPM(const char* filename, bool fullColour = true);
	int writeBayer(const char* filename);
private:
	int flipIndex(int row, int col);

	LibRaw *reader;
	int shift = 0;
};

