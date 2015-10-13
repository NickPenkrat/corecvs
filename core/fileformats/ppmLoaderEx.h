#pragma once
/*
	This class does not inherit from ppmLoader.
	Instead, it implements PGM (or even 3-channel PPM) reading with metadata support for further processing.
*/
#include <global.h>
#include "g12buffer.h"

namespace corecvs
{
	class PPMLoaderEx
	{
		bool readHeader(FILE *fp, unsigned long int *h, unsigned long int *w, unsigned short int *maxval, int *type);
	public:
		struct IMGData
		{
			double pre_mul[4];
			double cam_mul[4];
			double gamm[6];
			uint16_t black;
			uint16_t white;
			int filters;
			uint8_t bits;
		};

		PPMLoaderEx();
		G12Buffer** g12BufferCreateFromColoredPPM(const string &name);
		G12Buffer* loadBayer(const string& name);
		int save(string name, G12Buffer *buf);
		IMGData* getMetadata();
		G12Buffer* getBayer();

	private:
		char* nextLine(FILE *fp, int sz = 255);
		void addParam(char* param, double* values, int length);
		IMGData *metadata;
		G12Buffer* bayer;
	};
}