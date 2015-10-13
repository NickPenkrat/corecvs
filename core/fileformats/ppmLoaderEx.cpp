#include "ppmLoaderEx.h"

namespace corecvs
{
	PPMLoaderEx::PPMLoaderEx()
	{
		this->metadata = new IMGData();
	}

	// skips through comments and returns the first encountered non-comment line
	// while skipping through comments also reads all available metadata
	char* PPMLoaderEx::nextLine(FILE *fp, int sz)
	{
		char *buf = new char[sz];
		while (fread(buf, 1, 1, fp))
		{

			if (buf[0] != '#' && buf[0] != '\n' && buf[0] != '\r')
			{
				fseek(fp, -1, SEEK_CUR);
				if (sz > 0 && fgets(buf, sz, fp) == NULL)
				{
					printf("fgets() call failed %s:%d\n", __FILE__, __LINE__);
				}
				return buf;
			}
			else
			{
				fgets(buf, sz, fp);

				// try to read metadata
				char param[256];
				int n = 0;

				// read param name
				if (sscanf(buf, " @meta %s\t@values %d\t", param, &n))
				{
					char* numbers = strrchr(buf, '\t') + 1;
					double *values = new double[n];
					// read n param values
					for (int i = 0; i < n; i++)
					{
						sscanf(numbers, "%lf", &(values[i]));
						numbers = strchr(numbers, ' ') + 1;
					}
					addParam(param, values, n);
				}
				memset(buf, 0, sz);
			}
		}

		return nullptr;
	}

	void PPMLoaderEx::addParam(char* param, double* values, int length)
	{
		if (!strcmp(param, "pre_mul") && length <= 4)
			memcpy(metadata->pre_mul, values, length*sizeof(values[0]));

		if (!strcmp(param, "cam_mul") && length <= 4)
			memcpy(metadata->cam_mul, values, length*sizeof(values[0]));

		if (!strcmp(param, "gamm") && length <= 6)
			memcpy(metadata->gamm, values, length*sizeof(values[0]));

		if (!strcmp(param, "type") && length)
			metadata->filters = values[0];

		if (!strcmp(param, "black") && length)
			metadata->black = values[0];

		if (!strcmp(param, "white") && length)
			metadata->white = values[0];
		delete[] values;
	}

	bool PPMLoaderEx::readHeader(FILE *fp, unsigned long int *h, unsigned long int *w, unsigned short int *maxval, int *type)
	{
		char* header = nextLine(fp); // skip comments and read next line

		//Check the PPM file Type P5 or P6
		if ((header[0] != 'P') || ((header[1] != '5') && (header[1] != '6')))
		{
			printf("Image is not a supported PPM\n");
			return false;
		}
		*type = header[1] - '0';

		header = nextLine(fp);

		if (sscanf(header, "%lu%lu", w, h) != 2)
		{
			if (sscanf(header, "%lu", w) != 1)
			{
				printf("Image dimensions could not be read from line %s\n", header);
				return false;
			}
			else
			{
				header = nextLine(fp);
				if (sscanf(header, "%lu", h) != 1) // can be gotten rid of but i don't want to use a goto... TODO: think again
				{
					printf("Image dimensions could not be read from line %s\n", header);
					return false;
				}
			}
		}

		header = nextLine(fp);

		if (sscanf(header, "%hu", maxval) != 1)
		{
			printf("Image metric could not be read form line %s\n", header);
			return false;
		}

		// we assume that no comments can exist after the last header line

		//DOTRACE(("Image is P6 PPM [%lu %lu] max=%u\n", *h, *w, *maxval));
		return true;
	}

	G12Buffer** PPMLoaderEx::g12BufferCreateFromColoredPPM(const string& name)
	{
		G12Buffer **result = new G12Buffer*[3];
		FILE      *fp = NULL;
		uint8_t   *charImage = NULL;


		//PPM Headers Variable Declaration
		unsigned long int i, j;
		unsigned long int h, w;
		int type;
		unsigned short int maxval;
		int shiftCount = 0;

		//Open file for reading in Binary Mode
		fp = fopen(name.c_str(), "rb");

		if (fp == NULL)
		{
			printf("Image %s does not exist \n", name.c_str());
			return NULL;
		}

		if (!this->readHeader(fp, &h, &w, &maxval, &type))
		{
			return NULL;
		}

		if (type < 5 || type > 6)
			return NULL;

		result[0] = new G12Buffer(h, w, false);
		result[1] = new G12Buffer(h, w, false);
		result[2] = new G12Buffer(h, w, false);

		int size = (type == 6 ? 3 : 1) * (maxval < 256 ? 1 : 2) * w * h;
		charImage = new uint8_t[size];

		if (fread(charImage, 1, size * sizeof(uint8_t), fp) == 0)
		{
			printf("fread() call failed %s:%d\n", __FILE__, __LINE__);
			goto done;
		}

		if (maxval <= 255)
		{

			if (type == 5)
				for (i = 0; i < h; i++)
					for (j = 0; j < w; j++)
					{
						result[0]->element(i, j) = (charImage[i * w + j]) << 4;
						result[1]->element(i, j) = result[0]->element(i, j);
						result[2]->element(i, j) = result[0]->element(i, j);
					}

			if (type == 6)
				for (i = 0; i < h; i++)
					for (j = 0; j < w * 3; j += 3)
					{
						result[0]->element(i, j / 3) = (charImage[i * w * 3 + j]) << 4;
						result[1]->element(i, j / 3) = (charImage[i * w * 3 + j + 1]) << 4;
						result[2]->element(i, j / 3) = (charImage[i * w * 3 + j + 2]) << 4;
					}

		}
		else
		{
			for (shiftCount = 0; (maxval >> shiftCount) > G12Buffer::BUFFER_MAX_VALUE; shiftCount++);

			if (type == 5)
				for (i = 0; i < h; i++)
				{
					for (j = 0; j < w * 2; j += 2)
					{
						int offset = i * w * 2 + j;
						result[0]->element(i, j / 2) = ((charImage[offset + 1]) << 8 |
							(charImage[offset])) >> shiftCount;
						result[1]->element(i, j / 2) = result[0]->element(i, j / 2);
						result[2]->element(i, j / 2) = result[0]->element(i, j / 2);

						//ASSERT_FALSE((result[0]->element(i, j) >= (1 << G12Buffer::BUFFER_BITS)), "Internal error in image loader\n");
					}
				}

			if (type == 6)
				for (i = 0; i < h; i++)
				{
					for (j = 0; j < w * 6; j += 6)
					{
						int offset = i * w * 6 + j;
						result[0]->element(i, j / 6) = ((charImage[offset + 4]) << 8 |
							(charImage[offset + 0])) >> shiftCount;
						result[1]->element(i, j / 6) = ((charImage[offset + 2]) << 8 |
							(charImage[offset + 3])) >> shiftCount;
						result[2]->element(i, j / 6) = ((charImage[offset + 4]) << 8 |
							(charImage[offset + 5])) >> shiftCount;

						//ASSERT_FALSE((toReturn->element(i, j) >= (1 << G12Buffer::BUFFER_BITS)), "Internal error in image loader\n");
					}
				}
		}

	done:
		if (fp != NULL)
			fclose(fp);
		if (charImage != NULL)
			delete[] charImage;
		return result;
	}

	G12Buffer* PPMLoaderEx::loadBayer(const string& name)
	{
		FILE      *fp = NULL;
		uint8_t   *charImage = NULL;

		//PPM Headers Variable Declaration
		unsigned long int i, j;
		unsigned long int h, w;
		int type;
		unsigned short int maxval;
		int shiftCount = 0;

		//Open file for reading in Binary Mode
		fp = fopen(name.c_str(), "rb");

		if (fp == NULL)
		{
			printf("Image %s does not exist \n", name.c_str());
			return NULL;
		}

		if (!this->readHeader(fp, &h, &w, &maxval, &type))
		{
			return NULL;
		}

		if (type != 5)
			return NULL;

		this->metadata->bits = 1;
		while (maxval >> this->metadata->bits)
			this->metadata->bits++;

		this->bayer = new G12Buffer(h, w, false);

		uint size = (maxval < 0x100 ? 1 : 2) * w * h;
		charImage = new uint8_t[size];

		if (fread(charImage, 1, size, fp) == 0)
		{
			printf("fread() call failed %s:%d\n", __FILE__, __LINE__);
			goto done;
		}

		if (maxval <= 0xff)
		{

			for (i = 0; i < h; i++)
				for (j = 0; j < w; j++)
				{
					this->bayer->element(i, j) = (charImage[i * w + j]);
				}
		}
		else
		{
			for (shiftCount = 0; (maxval >> shiftCount) > G12Buffer::BUFFER_MAX_VALUE; shiftCount++);

			for (i = 0; i < h; i++)
			{
				for (j = 0; j < w * 2; j += 2)
				{
					int offset = i * w * 2 + j;
					this->bayer->element(i, j / 2) = ((charImage[offset + 0]) << 8 |
						(charImage[offset + 1])) >> shiftCount;

					//ASSERT_FALSE((result[0]->element(i, j) >= (1 << G12Buffer::BUFFER_BITS)), "Internal error in image loader\n");
				}
			}

		}

	done:
		if (fp != NULL)
			fclose(fp);
		if (charImage != NULL)
			delete[] charImage;
		return this->bayer;

	}

	PPMLoaderEx::IMGData* PPMLoaderEx::getMetadata()
	{
		return metadata;
	}

	G12Buffer* PPMLoaderEx::getBayer()
	{
		return this->bayer;
	}

	float mul(float(*A)[3], uint8_t *B, int x, int y, int width)
	{
		float res = 0;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
			{
				int index = (y + i)*width + (x + j);
				res += A[i][j] * B[index];
			}
		return res;
	}

	int PPMLoaderEx::save(string name, G12Buffer *buf)
	{
		return -1; //PPMLoader().save(name, buf);;
	}

}