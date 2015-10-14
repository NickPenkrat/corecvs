/*
	CR2 Bayer extractor console tool.
	Internally uses LibRaw.

*/

#include <queue>
#include <iostream>
#include "cr2reader.h"

using std::cout;
using std::endl;
using std::string;

void usage(char *argv[])
{
	cout << "Raw CR2 to PPM converter. Usage:" << endl
		<< argv[0] << " [-rp] [-b number] file1.cr2 file2.cr2" << endl
		<< "-r\tforce 8-bit PPM format for more than 8-bit data" << endl
		<< "-b N\tuse N-bit PPM format, where N is from 1 to 16" << endl
		<< "-p\toutput dcraw-proccessed image instead of Bayer sensor data" << endl;
}

int main(int argc, char *argv[])
{
	CR2Reader rdr;
	enum params
	{
		quality,
		bps,
	};
	std::queue<params> cmdline;
	if (argc > 1)
	{
		// if we need additional parameters, make this a struct or binary enum
		bool fullcolour = true;
		bool writebayer = true;
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
					rdr.setBPP(num);
					cmdline.pop();
				}
				else if (cmdline.size() && cmdline.front() == quality)
				{
					rdr.setQuality(num);
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
					case 'p':
						writebayer = false;
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
				if (cmdline.size())
				{
					if (cmdline.front() == bps)
						cout << "Missing value for parameter -b. See --usage for help." << endl;
					else if (cmdline.front() == quality)
						cout << "Missing value for -q. See --usage for help." << endl;
					return 1;
				}
				if (int error = rdr.open(argv[c]))
				{
					cout << "Could not open " << argv[c] << endl;
					return error;
				}
				if (writebayer)
				{
					rdr.writeBayer((string(argv[c]) + ".pgm").c_str());
				}
				else
				{
					rdr.processDCRaw();
					rdr.writePPM((string(argv[c]) + ".ppm").c_str(), fullcolour);
				}
			}
		}
	}
	else
	{
		usage(argv);
	}
}
