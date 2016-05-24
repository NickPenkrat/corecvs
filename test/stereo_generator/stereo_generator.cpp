/*
 * This test undistorts images from calibrationJob with estimated params
 */
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <type_traits>

#include "jsonSetter.h"
#include "jsonGetter.h"
#include "abstractPainter.h"
#include "fixtureScene.h"

#ifdef WITH_TBB
#include <tbb/tbb.h>
#endif
    
std::string scene  ="foo.json";
std::string prefix ="roof_v1_SP";
std::string postfix=".jpg";


std::tuple<corecvs::Matrix33, corecvs::Matrix33, corecvs::Matrix33, corecvs::Vector3dd, double> getRectifyingTransform(corecvs::FixtureCamera &camL, corecvs::FixtureCamera &camR, const corecvs::Matrix33 &newK)
{
	auto F = camL.fundamentalTo(camR);
	F /= F.a(2, 2);
	// 1. camL goes to K[I|0]
	auto R1 = camL.extrinsics.orientation.conjugated(),
		 R0 = camL.extrinsics.orientation;
	auto Tl = camL.extrinsics.position;
	//std::cout << R0 << std::endl;
	camR.extrinsics.orientation = camR.extrinsics.orientation ^ R1;
	camR.extrinsics.position = R0 * (camL.extrinsics.position - camR.extrinsics.position);

	camL.extrinsics.orientation = camL.extrinsics.orientation ^ R1;
	camL.extrinsics.position = R0 * camL.extrinsics.position - R0 * camL.extrinsics.position;
	auto F2 = camL.fundamentalTo(camR);
	F2 /= F2.a(2, 2);
	std::cout << "F-F2: " << (F - F2).frobeniusNorm() << std::endl;

	// 2. Now we want camR go to K[R|(T>0, 0, 0)]
	auto T = camR.extrinsics.position;
	auto e1 = T.normalised(),
		 e2 = corecvs::Vector3dd(-T[1], T[0], 0.0).normalised();
	auto e3 = e1 ^ e2;
	auto R = corecvs::Matrix33::FromRows(e1, e2, e3);

	auto Q = corecvs::Quaternion::FromMatrix(R);
	camL.extrinsics.orientation = Q.conjugated();
	camR.extrinsics.orientation = camR.extrinsics.orientation ^ Q.conjugated();
	camR.extrinsics.position = R * camR.extrinsics.position;

	auto F3 = camL.fundamentalTo(camR);
	F3 /= F3.a(2, 2);
	std::cout << "F-F3: " << (F - F3).frobeniusNorm() << std::endl;
	std::cout << camR.extrinsics.position << std::endl;

	// 3. Now all we have to do -- is to combine this stuff into rectifying homographies
	//    and point transformation.
	std::tuple<corecvs::Matrix33, corecvs::Matrix33, corecvs::Matrix33, corecvs::Vector3dd, double> res;
	// baseline
	std::get<4>(res) = !camR.extrinsics.position;
	// X1 = Rl * (X - Cl)
	// X2 = R * X1
	// X2 = R*Rl*X - Rl*R*Cl
	// X = Rl^tR^tX2+Cl
	std::get<3>(res) = Tl;
	std::get<2>(res) = (R1 ^ Q).toMatrix();
	std::get<1>(res) = (newK * camR.extrinsics.orientation.conjugated().toMatrix() * camR.intrinsics.getKMatrix33()).inv();
	std::get<0>(res) = (newK * camL.extrinsics.orientation.conjugated().toMatrix() * camL.intrinsics.getKMatrix33()).inv();

	return res;
}

void createRectified(corecvs::CameraFixture *fl, corecvs::FixtureCamera *cl, corecvs::CameraFixture *fr, corecvs::FixtureCamera *cr, const corecvs::Matrix33 &Kn, const corecvs::Vector2dd &size)
{
	std::cout << "Trying to rectify " << fl->name << cl->nameId << " and " << fr->name << cr->nameId << std::endl;
	auto camL = fl->getWorldCamera(cl),
	     camR = fr->getWorldCamera(cr);
	auto rt = getRectifyingTransform(camL, camR, Kn);
	auto dl = corecvs::RadialCorrection(camL->distortion).getUndistortionTransformation(camL->intrinsics.size, camL->intrinsics.undistortedSize, 0.25, false);	
	auto dr = corecvs::RadialCorrection(camL->distortion).getUndistortionTransformation(camL->intrinsics.size, camL->intrinsics.undistortedSize, 0.25, false);

	std::stringstream ssL, ssR;
	ssL << prefix << fl->name << cl->nameId << postfix;


	std::unique_ptr<corecvs::RGB24Buffer> imgL(QTRGB24Loader().load(ssL.str())),
	                                      imgR(QTRGB24Loader().load(ssL.str()));
	
}


// I do not want to parse names ('cause I accidentially
int main(int argc, char **argv)
{
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
	corecvs::Vector2dd sz(maxW, maxH);

	for (auto& fl: fs.fixtures())
		for (auto& cl: fl->cameras)
			for (auto& fr: fs.fixtures())
			{
				if (fl == fr)
					continue;
				for (auto& cr: fr->cameras)
				{
					createRectified(fl, cl, fr, cr, Kn, sz);
				}
			}
    return 0;
}
