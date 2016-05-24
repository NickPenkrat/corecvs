/*
 * This test undistorts images from calibrationJob with estimated params
 */
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <algorithm>

#include "jsonSetter.h"
#include "jsonGetter.h"
#include "abstractPainter.h"
#include "fixtureScene.h"

#ifdef WITH_TBB
#include <tbb/tbb.h>
#endif

std::pair<corecvs::Matrix33, corecvs::Matrix33> getRectifyingTransform(corecvs::FixtureCamera &camL, corecvs::FixtureCamera &camR, const corecvs::Matrix33 &newK)
{
	// 1. camL goes to K[I|0]
	camR.extrinsics.orientation ^= camL.extrinsics.orientation.conjugated();
	camR.extrinsics.position -= camL.extrinsics.orientation * camL.extrinsics.position;
}

// I do not want to parse names ('cause I accidentially
int main(int argc, char **argv)
{
    std::string scene  ="foo.json";
    std::string prefix ="roof_v1_SP";
	std::string postfix=".jpg";

	if (argc < 4)
	{
		std::cout << "Usage: " << argv[0] << " scene.json prefix postfix" << std::endl;
		return -1;
	}
	scene  =argv[1];
	prefix =argv[2];
	postfix=argv[3];

	corecvs::FixtureScene fs;	
	JSONGetter getter(scene.c_str());
	getter.visit(fs, "scene");

	std::vector<double> focals;
	double maxW = 0.0, maxH = 0.0;

	for (auto& f: fs.fixtures())
		for (auto& c: f->cameras)
		{
			std::cout << "Will use image " << prefix << f->name << c->nameId << postfix << std::endl;
			maxW = std::max(maxW, c->intrinsics.size[0]);
			maxH = std::max(maxH, c->intrinsics.size[1]);
			focals.push_back(c->intrinsics.focal[0]);
		}
	std::sort(focals.begin(), focals.end());
	maxW *= 1.1;
	maxH *= 1.1;
	double focal = (focals[focals.size() / 2 - 1] + focals[focals.size() / 2] + focals[focals.size() / 2 + 1]) / 3.0;
	std::cout << "New cameras will all have focal length " << focal << " and size " << maxH << "x" << maxW << " pixels" << std::endl;

	for (auto& fl: fs.fixtures())
		for (auto& c1: fl->cameras)
			for (auto& fr: fs.fixtures())
			{
				if (fl == fr)
					continue;
				for (auto& cr: fr->cameras)
				{
					std::cout << "Trying to rectify " << fl->name << c1.nameId << " and " << fr->name << c2.nameId << std::endl;
				}
			}
    return 0;
}
