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
    int processDCRaw(bool noScale = true);
    void fakeimg(G12Buffer *img);

    G12Buffer *getBayer(bool shifted = true);
    double *getGamm();

    void setBPP(uint bits);
    void setQuality(uint quality);

    int writePPM(const char* filename, bool fullColour = true);
    int writeBayer(const char* filename);

	~CR2Reader();
private:
	int flipIndex(int row, int col);
	void histUpdate(int i, int j, uint16_t val);
	uint64_t **hist = 0;
	LibRaw *reader;
	int shift = 0;
};

