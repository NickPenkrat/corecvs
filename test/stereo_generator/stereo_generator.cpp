/*
 * This test undistorts images from calibrationJob with estimated params
 */
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <type_traits>

#include "qtFileLoader.h"
#include "jsonSetter.h"
#include "jsonGetter.h"
#include "abstractPainter.h"
#include "fixtureScene.h"
#include "lmDistortionSolver.h"
#include "displacementBuffer.h"

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


	// 3. Now we rotate all the world till [t,0,0] and dir of first camera are in y=0
	auto d = camL.rayFromPixel(camL.intrinsics.size/2.0).a.normalised();
	d /= std::sqrt(d[1] * d[1] + d[2] * d[2]);
	corecvs::Matrix33 RD(1.0,  0.0,  0.0,
                         0.0, d[2], d[1],
			             0.0,-d[1], d[2]);
	auto QD = corecvs::Quaternion::FromMatrix(RD);
	camL.extrinsics.orientation = camL.extrinsics.orientation ^ QD.conjugated();
	camR.extrinsics.orientation = camR.extrinsics.orientation ^ QD.conjugated();
	auto F4 = camL.fundamentalTo(camR);
	F4 /= F4.a(2, 2);
	std::cout << "F-F4: " << (F - F4).frobeniusNorm() << std::endl;
			            


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
	std::get<1>(res) = (newK * camR.extrinsics.orientation.conjugated().toMatrix() * camR.intrinsics.getKMatrix33().inv());
	std::get<0>(res) = (newK * camL.extrinsics.orientation.conjugated().toMatrix() * camL.intrinsics.getKMatrix33().inv());

	auto ff =  std::get<0>(res).inv().transposed()*F*std::get<1>(res).inv();
	double maxv = 0.0;
	for (int i = 0; i < 9; ++i) maxv = std::max(maxv, ff.a(i % 3, i / 3));
	std::cout << ff / maxv << std::endl;

	return res;
}

void createRectified(corecvs::CameraFixture *fl, corecvs::FixtureCamera *cL, corecvs::CameraFixture *fr, corecvs::FixtureCamera *cR, const corecvs::Matrix33 &Kn, const corecvs::Vector2dd &size)
{
	std::cout << "Trying to rectify " << fl->name << cL->nameId << " and " << fr->name << cR->nameId << std::endl;
	if (std::acos(fl->rayFromPixel(cL, size/2.0).a.normalised() & fr->rayFromPixel(cR, size/2.0).a.normalised()) * 180.0 / M_PI > 15.0 ||
		std::abs(std::acos(std::abs(fl->rayFromPixel(cL, size/2.0).a.normalised() & (fl->location.shift-fr->location.shift).normalised())) * 180.0 / M_PI - 90.0) > 15.0 ||
		std::abs(std::acos(std::abs(fr->rayFromPixel(cR, size/2.0).a.normalised() & (fl->location.shift-fr->location.shift).normalised())) * 180.0 / M_PI - 90.0) > 15.0)
	{
		std::cout << "Too big angle diff" << std::endl;
		return;
	}
	auto cl = fl->getWorldCamera(cL),
	     cr = fr->getWorldCamera(cR);
	auto rt = getRectifyingTransform(cl, cr, Kn);
	auto dl = corecvs::RadialCorrection(cl.distortion).getUndistortionTransformation(cl.intrinsics.size, cl.intrinsics.distortedSize, 0.25, false);	
	auto dr = corecvs::RadialCorrection(cr.distortion).getUndistortionTransformation(cr.intrinsics.size, cr.intrinsics.distortedSize, 0.25, false);

	std::stringstream ssL, ssR;
	ssL << prefix << fl->name << cl.nameId << postfix;
	ssR << prefix << fr->name << cr.nameId << postfix;


	std::unique_ptr<corecvs::RGB24Buffer> imgL(QTRGB24Loader().load(ssL.str())),
	                                      imgR(QTRGB24Loader().load(ssR.str()));
	std::cout << imgL->w << "x" << imgL->h << std::endl;
	std::cout << imgR->w << "x" << imgR->h << std::endl;
	std::unique_ptr<corecvs::RGB24Buffer> uL(imgL->doReverseDeformationBlTyped<corecvs::DisplacementBuffer>(&dl, cl.intrinsics.size[1], cl.intrinsics.size[0])),
	                                      uR(imgR->doReverseDeformationBlTyped<corecvs::DisplacementBuffer>(&dr, cr.intrinsics.size[1], cr.intrinsics.size[0]));
	corecvs::DisplacementBuffer pl(&std::get<0>(rt), size[1], size[0]),
                                pr(&std::get<1>(rt), size[1], size[0]);
	std::unique_ptr<corecvs::RGB24Buffer> iL(uL->doReverseDeformationBlTyped<corecvs::DisplacementBuffer>(&pl, cl.intrinsics.size[1], cl.intrinsics.size[0])),
	                                      iR(uR->doReverseDeformationBlTyped<corecvs::DisplacementBuffer>(&pr, cr.intrinsics.size[1], cr.intrinsics.size[0]));
	for (int i = 10; i < iL->h; i += 20)
		for (int j = 0; j < iL->w; ++j)
			iL->element(i, j) = iR->element(i, j) = corecvs::RGBColor(255, 0, 0);
    std::stringstream ssLr, ssRr;
    ssLr << fl->name << cl.nameId << fr->name << cr.nameId << "_rect_L.jpg";
    ssRr << fl->name << cl.nameId << fr->name << cr.nameId << "_rect_R.jpg";
    QTFileLoader().save(ssLr.str(), iL.get(), 100);
    QTFileLoader().save(ssRr.str(), iR.get(), 100);
	std::cout << ssLr.str() << ssRr.str() << std::endl;
	std::cout << iL->w << "x" << iL->h << std::endl;
	std::cout << iR->w << "x" << iR->h << std::endl;
	std::cout << std::get<0>(rt) << std::get<1>(rt) << std::endl << std::endl;

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
