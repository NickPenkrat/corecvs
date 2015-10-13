#include "cr2reader.h"

// the define fixes winsock2 conflicts for inclusion
#define NO_WINSOCK
#include <libraw.h>
#undef NO_WINSOCK

CR2Reader::CR2Reader()
{
	reader = new LibRaw;
	reader->imgdata.params.user_qual = 3; // default quality
	reader->imgdata.params.output_bps = 12; // default bit depth
}

CR2Reader::CR2Reader(const char *filename) : CR2Reader()
{
	open(filename);
}

int CR2Reader::open(const char *filename)
{
	std::string tmp = std::string(filename);
	int result = reader->open_file(std::string(filename).c_str());
	result |= reader->unpack();
	reader->imgdata.params.use_auto_wb = 0;
	reader->imgdata.params.use_camera_wb = 1;

	int maxval = 0;
	shift = 0;

	for (int i = 0; i < reader->imgdata.sizes.raw_height*reader->imgdata.sizes.raw_width; i++)
		if (reader->imgdata.rawdata.raw_image[i]>maxval)
			maxval = reader->imgdata.rawdata.raw_image[i];

	while (maxval >> shift > (1 << reader->imgdata.params.output_bps) - 1)
		shift++;

	return result;
}

void CR2Reader::setBPP(uint depth)
{
	reader->imgdata.params.output_bps = depth;
}

double* CR2Reader::getGamm()
{
	return reader->imgdata.params.gamm;
}

void CR2Reader::setQuality(uint quality)
{
	reader->imgdata.params.user_qual = quality;
}

G12Buffer *CR2Reader::getBayer(bool shifted)
{
	G12Buffer *result = new G12Buffer(reader->imgdata.sizes.height, reader->imgdata.sizes.width);
	int locShift = shifted * shift;
	for (int i = 0; i < reader->imgdata.sizes.height; i++)
		for (int j = 0; j < reader->imgdata.sizes.width; j++)
		{
			int offset = (i + reader->imgdata.sizes.top_margin)*reader->imgdata.sizes.raw_width + (j + reader->imgdata.sizes.left_margin);
			result->element(i, j) = reader->imgdata.rawdata.raw_image[offset] >> locShift;
		}
	return result;
}

G12Buffer** CR2Reader::getChannels()
{
	G12Buffer **result = new G12Buffer*[reader->imgdata.idata.colors];
	for (int i = 0; i < reader->imgdata.idata.colors; i++)
		result[i] = getChannel(i);
	return result;
}

G12Buffer* CR2Reader::getChannel(int channel)
{
	G12Buffer *b = new G12Buffer(reader->imgdata.sizes.height, reader->imgdata.sizes.width);
	for (int i = 0; i < reader->imgdata.sizes.height; i++)
		for (int j = 0; j < reader->imgdata.sizes.width * 4; j += 4)
		{
			b->element(i, j / 4) = reader->imgdata.image[0][i*reader->imgdata.sizes.width * 4 + j] >> 4;
		}
	return b;
}

int CR2Reader::flipIndex(int row, int col)
{
	if (reader->imgdata.sizes.flip & 4) { row = row + col; col = row - col; row = row - col; };
	if (reader->imgdata.sizes.flip & 2) row = reader->imgdata.sizes.iheight - 1 - row;
	if (reader->imgdata.sizes.flip & 1) col = reader->imgdata.sizes.iwidth - 1 - col;
	return row * reader->imgdata.sizes.iwidth + col;
}

int isBigEndian()
{
	union {
		uint32_t i;
		char c[4];
	} bint = { 0x01020304 };

	return bint.c[0] == 1;
}

int CR2Reader::writePPM(const char* filename, bool fullcolour)
{
	int h = reader->imgdata.sizes.height;
	int w = reader->imgdata.sizes.width;

	if (reader == NULL || reader->imgdata.params.output_bps > 16 || reader->imgdata.params.output_bps < 1 || !h || !w)
		return -1;

	FILE *fp;
	fp = fopen(filename, "wb");

	if (fp == NULL)
	{
		printf("Image %s could not be written \n", filename);
		return -1;
	}

	fprintf(fp, "P6\n");
	fprintf(fp, "############################################\n");
	fprintf(fp, "# The original file was a CR2.\n");
	fprintf(fp, "# Bit depth: %i bits per channel.\n", reader->imgdata.params.output_bps);

	/// \todo TODO: Add some metadata saving

	fprintf(fp, "############################################\n");
	fprintf(fp, "%d %d\n", w, h);

	cout << "Writing " << reader->imgdata.params.output_bps * 3 << " bits per pixel with quality level " << reader->imgdata.params.user_qual << "..." << std::endl;

	// calculate bitwise shift for our custom colour depth
	if (reader->imgdata.params.output_bps <= 8 || fullcolour)
		shift = 16 - reader->imgdata.params.output_bps;
	else
		shift = 8;

	fprintf(fp, "%d\n", (1 << (16 - shift)) - 1);

	uint8_t *img8 = new uint8_t[3 * 2 * w * h];

	// 16bit buffer is just the 8bit buffer with a different pointer
	ushort *img16 = (ushort*)img8;

	// to account for different image matrix configurations, we use libraw-calculated steps
	int soff = flipIndex(0, 0);
	int cstep = flipIndex(0, 1) - soff;
	int rstep = flipIndex(1, 0) - flipIndex(0, reader->imgdata.sizes.width);
	int colors = reader->imgdata.idata.colors;
	int width = reader->imgdata.sizes.width;
	int height = reader->imgdata.sizes.height;
	int t_white = 0x2000;

	// white level percentile (currently unused as we have no histogram whatsoever)
	//perc = width * height * reader->imgdata.params.auto_bright_thr;

	// calculate gamma correction
	reader->gamma_curve(reader->imgdata.params.gamm[0], reader->imgdata.params.gamm[1], 2, (t_white << 3) / reader->imgdata.params.bright);
	reader->imgdata.sizes.iheight = height;
	reader->imgdata.sizes.iwidth = width;

	int bytesperpixel = fullcolour ? ((reader->imgdata.params.output_bps + 7) / 8) : 1;

	// the following code was taken from libraw
	for (int row = 0; row < height; row++, soff += rstep)
	{
		for (int col = 0; col < width; col++, soff += cstep)
			if (bytesperpixel == 1)
				for (int c = 0; c < colors; c++)
					img8[col*colors + c] = reader->imgdata.color.curve[reader->imgdata.image[soff][c]] >> shift;
			else for (int c = 0; c < colors; c++)
				img16[col*colors + c] = reader->imgdata.color.curve[reader->imgdata.image[soff][c]] >> shift;
		if (bytesperpixel == 2 && !isBigEndian())
			swab((char*)img16, (char*)img16, width*colors * 2);
		fwrite(img8, colors * bytesperpixel, width, fp);
	}

	delete[] img8;
	fclose(fp);
	return 0;
}

int CR2Reader::writeBayer(const char* filename)
{
	if (reader == NULL || reader->imgdata.params.output_bps > 16)
		return -1;

	int h = reader->imgdata.sizes.height;
	int w = reader->imgdata.sizes.width;

	FILE *fp;
	fp = fopen(filename, "wb");

	if (fp == NULL)
	{
		printf("Image %s could not be written \n", filename);
		return -1;
	}

	// get shifted bayer
	G12Buffer *b = getBayer(true);
	uint16_t *img16 = new uint16_t[b->w*b->h];
	uint8_t *img8 = (uint8_t*)img16;

	uint16_t maxval = 0;
	// shift already applied by getBayer, we got our N-bit data
	for (int i = 0; i < b->h; i++)
		for (int j = 0; j < b->w; j++)
		{
			if (maxval < b->element(i, j))
				maxval = b->element(i, j);
			img16[i*b->w + j] = b->element(i, j);
		}

	// TODO: rewrite the following so as not to use winsock
	if (!isBigEndian())
		swab((char*)img16, (char*)img16, b->w * b->h * 2);

	fprintf(fp, "P5\n");
	fprintf(fp, "############################################\n");
	fprintf(fp, "# Bit depth: %i bits per channel.\n", reader->imgdata.params.output_bps);

	fprintf(fp, "# @meta %s\t@values 4\t%f %f %f %f\n", "pre_mul", reader->imgdata.color.pre_mul[0], reader->imgdata.color.pre_mul[1], reader->imgdata.color.pre_mul[2], reader->imgdata.color.pre_mul[3]);
	fprintf(fp, "# @meta %s\t@values 4\t%f %f %f %f\n", "cam_mul", reader->imgdata.color.cam_mul[0], reader->imgdata.color.cam_mul[1], reader->imgdata.color.cam_mul[2], reader->imgdata.color.cam_mul[3]);
	fprintf(fp, "# @meta %s\t@values 6\t%f %f %f %f %f %f\n", "gamm", reader->imgdata.params.gamm[0], reader->imgdata.params.gamm[1], reader->imgdata.params.gamm[2], reader->imgdata.params.gamm[3], reader->imgdata.params.gamm[4], reader->imgdata.params.gamm[5]);
	fprintf(fp, "# @meta %s\t@values 1\t%d\n", "type", reader->imgdata.idata.filters);
	fprintf(fp, "# @meta %s\t@values 1\t%d\n", "white", maxval);
	fprintf(fp, "# @meta %s\t@values 1\t%d\n", "black", reader->imgdata.color.black >> shift);

	fprintf(fp, "############################################\n");
	fprintf(fp, "%d %d\n", w, h);
	fprintf(fp, "%d\n", (1 << reader->imgdata.params.output_bps) - 1);

	fwrite(img8, 2, b->w * b->h, fp);
	fclose(fp);
	return 0;
}

int CR2Reader::processDCRaw()
{
	return reader->dcraw_process();
}

CR2Reader::~CR2Reader()
{
}
