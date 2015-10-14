#pragma once
/*
	This class does not inherit from ppmLoader.
	Instead, it implements PGM (or even 3-channel PPM) reading with metadata support for further processing.
*/
#include "global.h"
#include <map>
#include "g12buffer.h"

namespace corecvs
{
	class PPMLoaderEx
	{
	public:
		typedef std::pair<string, double*> MetaValue;
		typedef std::map<string, double*> MetaData;

		PPMLoaderEx();
		G12Buffer** g12BufferCreateFromColoredPPM(const string &name);
		static MetaData nulldata;
		int loadBayer(const string& name, MetaData& metadata = nulldata);
		int save(string name, G12Buffer *buf);
		G12Buffer* getBayer();

	private:
		G12Buffer* bayer;

		bool readHeader(FILE *fp, unsigned long int *h, unsigned long int *w, unsigned short int *maxval, int *type, MetaData& metadata = nulldata);
		char* nextLine(FILE *fp, int sz, MetaData& metadata);
		//MetaData metadata;
	};
}