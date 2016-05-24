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
	auto R1 = camL.extrinsics.orientation.conjugated(),
		 R0 = camL.extrinsics.orientation;
	std::cout << R0 << std::endl;
	camR.extrinsics.orientation = camR.extrinsics.orientation ^ R1;
	camR.extrinsics.position = R0 * (camL.extrinsics.position - camR.extrinsics.position);

	camL.extrinsics.orientation = camL.extrinsics.orientation ^ R1;
	camL.extrinsics.position = R0 * camL.extrinsics.position - R0 * camL.extrinsics.position;

	// 2. Now we want camR go to K[R|(T>0, 0, 0)]
	auto T = camR.extrinsics.position;
	auto e1 = T.normalised(),
		 e2 = corecvs::Vector3dd(-T[1], T[0], 0.0).normalised();
	auto e3 = e1 ^ e2;
	std::cout << e1 << std::endl;
	auto R = corecvs::Matrix33::FromRows(e1, e2, e3);
	std::cout << R.det() << std::endl;
	std::cout << R * camR.extrinsics.position << std::endl;
	CORE_ASSERT_TRUE_S(R.det() > 0.0);
	return std::make_pair(R, R);
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

	corecvs::Matrix33 Kn(focal, 0.0, maxW / 2.0, 0.0, focal, maxH / 2.0, 0.0, 0.0, 1.0);

	for (auto& fl: fs.fixtures())
		for (auto& cl: fl->cameras)
			for (auto& fr: fs.fixtures())
			{
				if (fl == fr)
					continue;
				for (auto& cr: fr->cameras)
				{
					std::cout << fl->location.rotor << " " << cl->extrinsics.orientation << std::endl;
					std::cout << "Trying to rectify " << fl->name << cl->nameId << " and " << fr->name << cr->nameId << std::endl;
					auto camL = fl->getWorldCamera(cl),
						 camR = fr->getWorldCamera(cr);
					getRectifyingTransform(camL, camR, Kn);
				}
			}
    return 0;
}
