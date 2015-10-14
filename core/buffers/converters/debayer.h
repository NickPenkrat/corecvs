#pragma once

#include "g12Buffer.h"
#include "ppmLoaderEx.h"

using namespace std;
using corecvs::G12Buffer;
using corecvs::PPMLoaderEx;

class Debayer
{
public:

	static Debayer* FromFile(const string& filename);

	Debayer(G12Buffer *bayer, PPMLoaderEx::MetaData &data);

	G12Buffer** linear();
	G12Buffer** nearest();

	int writePPM(string filename, int depth = 16);
	~Debayer();
private:
	G12Buffer* bayer;
	G12Buffer** out;
	PPMLoaderEx *loader;
	PPMLoaderEx::MetaData metadata;

	double* scale_colors(bool highlight = false);
	uint16_t* gamma_curve(int mode, int imax, int depth = 16);
};

