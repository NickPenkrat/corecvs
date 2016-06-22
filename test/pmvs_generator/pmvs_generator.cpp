#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <random>

#include "calibrationJob.h"
#include "jsonSetter.h"
#include "jsonGetter.h"
#include "fixtureScene.h"
#include "matrix44.h"
#include "qtFileLoader.h"

int main(int argc, char **argv)
{
	if (argc < 4)
	{
		std::cout << "Usage: " << argv[0] << " scene.json image_prefix image_postfix [dir]" << std::endl;
		return -1;
	}

	std::string scene(argv[1]), prefix(argv[2]), postfix(argv[3]);
	JSONGetter getter(scene);
	corecvs::FixtureScene fs;
	getter.visit(fs, "scene");

	std::string maxNameId = "6";
	std::string dir = "./";
	if (argc > 4)
		dir = std::string(argv[4]) + "/";

	int validImages = 0;
	std::map<int, corecvs::Matrix44> P;
	std::map<int, std::string> I;
	std::map<int, WPP> wpp;
	std::mt19937 rng((std::random_device())());
	std::uniform_real_distribution<double> runif(-100, 100);

	for (auto& f: fs.fixtures())
	{
		for (auto& c: f->cameras)
		{
			CORE_ASSERT_TRUE_S(c->nameId.size());
			if (c->nameId >= maxNameId)
				continue;
			auto cc = f->getWorldCamera(c);
			P[validImages] = corecvs::Matrix44(cc.intrinsics.getKMatrix33())*(corecvs::Matrix44)cc.extrinsics.toAffine3D().inverted();
			corecvs::Vector3dd vec(runif(rng), runif(rng), runif(rng));
			auto proj = cc.project(vec);
			auto Proj = P[validImages] * vec;
			corecvs::Vector2dd proj_(Proj[0]/Proj[2], Proj[1]/Proj[2]);
			CORE_ASSERT_TRUE_S((!(proj_ - proj)) < 1e-3);
			I[validImages] = prefix + f->name + c->nameId + postfix;
			wpp[validImages] = WPP(f, c);
			validImages++;
		}
	}

	const std::string 
#ifndef WIN32
					  MKD_CMD = "mkdir -p "
#else
					  MKD_CMD = "mkdir /Y"
#endif
		             ,IMG_DIR = "/visualize/",
		              TXT_DIR = "/txt/";
	system((MKD_CMD + dir + IMG_DIR).c_str());
	system((MKD_CMD + dir + TXT_DIR).c_str());

	corecvs::parallelable_for(0, validImages, [&](const corecvs::BlockedRange<int> &r)
			{
				for (int i = r.begin(); i < r.end(); ++i)
				{
					std::string img;
					std::stringstream ss;
					ss << std::setfill('0') << std::setw(8) << i;
					img = ss.str();

					std::string undistorted = dir + IMG_DIR + img + ".jpg",
					            pmatrix = dir + TXT_DIR + img + ".txt";

					std::ofstream of;
					of.open(pmatrix, std::ios_base::out);
					of << "CONTOUR" << std::endl;
					auto PP = P[i];
					for (int ii = 0; ii < 3; ++ii)
					{
						for (int jj = 0; jj < 4; ++jj)
							of << PP.a(ii, jj) << " ";
						of << std::endl;
					}

					auto cam = *wpp[i].v;
					auto disp = corecvs::RadialCorrection(cam.distortion).getUndistortionTransformation(cam.intrinsics.size, cam.intrinsics.distortedSize, 0.25, false);
					
					std::unique_ptr<corecvs::RGB24Buffer> src(QTRGB24Loader().load(I[i]));
					std::unique_ptr<corecvs::RGB24Buffer> res(src->doReverseDeformationBlTyped<corecvs::DisplacementBuffer>(&disp, cam.intrinsics.size[1], cam.intrinsics.size[0]));

					QTFileLoader().save(undistorted, res.get(), 100);
				}
			});

	std::vector<int> target, other;
	auto firstU = wpp.begin()->second.u;
	for (int i = 0; i < validImages; ++i)
		(wpp[i].u == firstU ? target : target).push_back(i);
//		(wpp[i].u == firstU ? target : other).push_back(i);
	
	std::ofstream of;
	of.open(dir + "/options.txt", std::ios_base::out);
	of << "timages " << target.size() << " ";
	for (auto& id: target)
		of << id << " ";
	of << std::endl;

	of << "oimages " << other.size() << " ";
	for (auto& id: other)
		of << id << " ";
	of << std::endl;

	of << "CPU 8" << std::endl << "useVisData 1" << std::endl;


	std::vector<std::vector<int>> visdata(validImages);
	for (auto& img: wpp)
	{
		std::map<int, int> currN;
		auto c = img.second;
		for (auto& f: fs.featurePoints())
		{
			std::set<int> currP;
			//std::cout << f->observations.size() << std::endl;
			for (auto& o: f->observations)
			{
//				std::cout << o.second.cameraFixture << ":";
				for (auto& ii: wpp)
					if (/*ii.second.u == o.second.cameraFixture &&*/ ii.second.v == o.second.camera)
					{
						currP.insert(ii.first);
						auto f1 = wpp[img.first].u, f2 = o.second.camera->cameraFixture;
						auto c1 = wpp[img.first].v, c2 = o.second.camera;
						auto cc1 = f1->getWorldCamera(c1), cc2 = f2->getWorldCamera(c2);
						auto px1 = cc1.intrinsics.size / 2, px2 = cc2.intrinsics.size / 2;
						auto r1 = cc1.rayFromPixel(px1).a, r2 = cc2.rayFromPixel(px2).a;
						if (std::acos(r1.normalised() & r2.normalised())*180.0/M_PI > 10.0)
							continue;
					}
			}
			if (!currP.count(img.first))
				continue;
			for (auto& id: currP)
				currN[id]++;
		}
		if (!currN.count(img.first))
			continue;
		for (auto& id : currN)
			if (id.first != img.first && id.second > 20)
				visdata[img.first].push_back(id.first);
//		std::cout << visdata[img.first].size() << std::endl;
	}

#if 1
	std::ofstream vis;
	vis.open(dir + "/vis.dat", std::ios_base::out);
	vis << "VISDATA" << std::endl << visdata.size() << std::endl;
	for (int i = 0; i < visdata.size(); ++i)
	{
		vis << i << " " << visdata[i].size() << " ";
		for (auto& id: visdata[i])
			vis << id << " ";
		vis << std::endl;
	}
#endif
    return 0;
}
