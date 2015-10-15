/*
Bayer to PPM converter

*/

#include <queue>
#include <iostream>
#include "ppmLoaderEx.h"
#include "converters/debayer.h"

using std::cout;
using std::endl;
using std::string;
using corecvs::PPMLoaderEx;

void usage(char *argv[])
{
	cout << "PGM Bayer to RGB PPM converter. Usage:" << endl
		<< argv[0] << " [-r] [-b number] [-q number] file1.pgm file2.pgm" << endl
		<< "-q N\tdemosaic quality:" << endl
		<< "\t0:\t nearest neighbour" << endl
		<< "\t1:\t bilinear" << endl
		<< "-b N\tuse N-bit PPM format, where N is from 1 to 16" << endl
		<< "-r\tforce 8-bit PPM format for more than 8-bit data; shortcut for -b 8" << endl;
}

int main(int argc, char *argv[])
{
	enum params
	{
		bps,
		quality,
	};

	PPMLoaderEx ldr;
	std::queue<params> cmdline;
	int user_quality = 0;

	if (argc > 1)
	{
		// if we need additional parameters, make this a struct or binary enum

		// write full colour (up to 16 bit) image
		bool fullcolour = true;
		for (int c = 1; c < argc; c++)
		{
			char arg[255];
			int num = atoi(argv[c]);

			if (cmdline.size())
			{
				if (cmdline.front() == bps)
				{
					if (num < 1 || num > 16)
					{
						cout << "Only values 1-16 are supported for bit depth." << endl;
						return 1;
					}
					//rdr.setBPP(num);
					cmdline.pop();
				}
				if (cmdline.front() == quality)
				{
					if (num < 0 || num > 1)
					{
						cout << "Only values 0-1 are supported for quality." << endl;
						return 1;
					}
					user_quality = num;
					cmdline.pop();
				}
				else
				{
					usage(argv);
					return 0;
				}
			}
			else
				if (argv[c][0] == '-')
				{
					int j = 1, k = 2;
					while (argv[c][j])
					{
						arg[0] = 0;
						switch (argv[c][j])
						{
						case 'r':
							fullcolour = false;
							break;
						case 'b':
							cmdline.push(bps);
							break;
						case 'q':
							cmdline.push(quality);
							break;
						case '-':
							while (argv[c][k])
								arg[k - 2] = argv[c][k++];
							arg[k - 2] = '\0';
							goto arghandle; // sorry
							break;
						default:
							cout << "Invalid parameter -" << argv[c][j] << ", use --usage to view help" << endl;
							break;
						}
						j++;
					}
				arghandle:
					if (strlen(arg))
						if (!strcmp(arg, "usage"))
						{
							usage(argv);
						}
						else
							cout << "Invalid parameter --" << arg << ", see --usage for help." << endl;
				}
				else
				{
					PPMLoaderEx::MetaData *metadata = new PPMLoaderEx::MetaData;
					if (ldr.loadBayer(argv[c], metadata))
					{
						cout << "Could not open " << argv[c] << "." << endl;
						return -1;
					}
					Debayer d(ldr.getBayer(), metadata);

					switch (user_quality)
					{
					case 0:
					default:
						cout << "Using Nearest Neighbour interpolation..." << endl;
						d.nearest();
						break;
					case 1:
						cout << "Using Bilinear interpolation..." << endl;
						d.linear();
						break;
					}
					d.writePPM(string(argv[c]) + ".ppm");
					cout << "Done." << endl;
				}
		}
	}
	else
	{
		usage(argv);
	}
}
