#pragma once

#include "g12Buffer.h"
#include "ppmLoaderEx.h"

using namespace std;
using corecvs::G12Buffer;
using corecvs::PPMLoaderEx;
class Debayer
{
private:
	// this type is used here quite often
	typedef PPMLoaderEx::MetaData MetaData;

	G12Buffer* bayer;
	G12Buffer** out;
	PPMLoaderEx *loader;
	MetaData *metadata_ptr;

	double* scale_colors(bool highlight = false, MetaData *metadata = nullptr);
	uint16_t* gamma_curve(int mode, int imax, int depth, MetaData *metadata = nullptr);
public:
	static Debayer* FromFile(const string& filename);

	Debayer(G12Buffer *bayer, MetaData *data = nullptr);

	G12Buffer** linear();
	G12Buffer** nearest();

	int writePPM(string filename, int depth = 16);
	~Debayer();
};

