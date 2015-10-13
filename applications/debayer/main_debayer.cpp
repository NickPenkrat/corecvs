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
		<< argv[0] << " [-rn] [-b number] file1.pgm file2.pgm" << endl
		<< "-r\tforce 8-bit PPM format for more than 8-bit data" << endl
		<< "-b N\tuse N-bit PPM format, where N is from 1 to 16" << endl;
}

int main(int argc, char *argv[])
{
	enum params
	{
		bps,
	};
	PPMLoaderEx ldr;
	std::queue<params> cmdline;

	if (argc > 1)
	{
		// if we need additional parameters, make this a struct or binary enum

		// write full colour (up to 16 bit) image
		bool fullcolour = true;
		for (int c = 1; c < argc; c++)
		{
			char arg[255];
			int num = atoi(argv[c]);

			if (num)
			{
				if (cmdline.size() && cmdline.front() == bps)
				{
					if (num < 1 || num > 16)
					{
						cout << "Only values 1-16 are supported for bit depth." << endl;
						return 1;
					}
					//rdr.setBPP(num);
					cmdline.pop();
				}
				else
				{
					usage(argv);
					return 0;
				}
			}

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
				if (cmdline.size())
				{
					if (cmdline.front() == bps)
						cout << "Missing value for parameter -b. See --usage for help." << endl;
					return 1;
				}
				ldr.loadBayer(argv[c]);
				Debayer d(ldr.getBayer(), ldr.getMetadata());
				d.nearest();
				d.writePPM("out.ppm");
			}
		}
	}
	else
	{
		usage(argv);
	}
}
