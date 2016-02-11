#include "sceneGenerator.h"
#include <random>


using namespace corecvs;

void SceneGenerator::generateScene()
{
	if (rfs) delete rfs;
	rfs = new ReconstructionFixtureScene();
	generateFixtures();
	generatePoints();
	generateMatches();
	rfs->placingQueue = rfs->fixtures;
	rfs->state = ReconstructionState::MATCHED;
}

void SceneGenerator::generateFixtures()
{
	double R = 20.0;
	double l = 0.0, r = N;
	for (int i = 0; i < 64; ++i)
	{
		double m = (l + r) / 2.0;
		int cnt = 0;
		for (int i = -N; i <= N; ++i)
			for (int j = -N; j <= N; ++j)
			{
				double x = j + (i % 2) * 0.5;
				double y = i * std::sqrt(3.0) / 2.0;
				if (x * x + y * y < m * m)
					cnt++;
			}
		if (cnt <= N)
			l = m;
		else
			r = m;
	}

	std::vector<corecvs::Vector2dd> fixtures;
	for (int i = -N; i <= N; ++i)
		for (int j = -N; j <= N; ++j)
		{
			double x = j + (i % 2) * 0.5;
			double y = i * std::sqrt(3.0) / 2.0;
			if (x * x + y * y <= r * r)
				fixtures.emplace_back(x * R, y * R);
		}
	CORE_ASSERT_TRUE_S(fixtures.size() >= N);
	std::sort(fixtures.begin(), fixtures.end(), [](const corecvs::Vector2dd &a, const corecvs::Vector2dd &b) { return !a < !b; });
	fixtures.resize(N);
	for (auto&f: fixtures)
	{
		std::cout << f << std::endl;
		generatePs(Vector3dd(f[0], f[1], 0), &f - &fixtures[0]);
	}


}

void SceneGenerator::generatePoints()
{
	std::cout << "PTGEN" << std::endl;
	std::random_device rd;
	std::mt19937 rng(rd());
	std::uniform_real_distribution<double> r3(rIn * rIn * rIn, rOut * rOut * rOut),
		                                  phi(0, 2.0 * M_PI),
		                                  uni(0, 1.0);
	std::normal_distribution<double> rnorm(0.0, sigmaProjPOI);
	for (int i = 0; i < M; ++i)
	{
		double r = std::cbrt(r3(rng)),
		      ph = phi(rng),
		       z = uni(rng) * 2  - 1.0;
		auto pt = Vector3dd(sqrt(1.0 - z * z) * sin(ph), sqrt(1.0 - z * z) * cos(ph), z) * r;
		int visible = 0;
		std::vector<WPP> vis;
		for (auto& f: rfs->fixtures)
			for (auto& c: f->cameras)
				if (f->isVisible(pt, c))
				{
					visible++;
					vis.emplace_back(f, c);
				}
		if (visible < poiMeasureLimit * N)
		{
			i--;
			continue;
		}
		auto p = rfs->createFeaturePoint();
		p->reprojectedPosition =  p->position = pt;
		p->color = corecvs::RGBColor(255, 0, 0);
		for (auto wpp: vis)
		{
			auto proj = wpp.u->project(pt, wpp.v) + Vector2dd(rnorm(rng), rnorm(rng));
			auto& obs = p->observations__[wpp];
			obs.camera = wpp.v;
			obs.cameraFixture = wpp.u;
			obs.featurePoint = p;
			obs.observation = proj;
		}
	}
}

void SceneGenerator::generateMatches()
{
	std::cout << "MGEN" << std::endl;
	std::random_device rd;
	std::mt19937 rng(rd());
	std::uniform_real_distribution<double> r3(rIn * rIn * rIn, rOut * rOut * rOut),
		                                  phi(0, 2.0 * M_PI),
		                                  uni(0, 1.0);
	std::normal_distribution<double> rnorm(0.0, sigmaProj);
	for (int i = 0; i < M; ++i)
	{
		double r = std::cbrt(r3(rng)),
		      ph = phi(rng),
		       z = uni(rng) * 2  - 1.0;
		auto pt = Vector3dd(sqrt(1.0 - z * z) * sin(ph), sqrt(1.0 - z * z) * cos(ph), z) * r;
		int visible = 0;
		std::vector<WPP> vis;
		for (auto& f: rfs->fixtures)
			for (auto& c: f->cameras)
				if (f->isVisible(pt, c))
				{
					visible++;
					vis.emplace_back(f, c);
				}
		if (visible < 3)
		{
			i--;
			continue;
		}
		int cnt = 3 + log(uni(rng))/log(1.0 - gamma);
		if (visible > cnt)
		{
			int idx = 0;
			for (int j = cnt; j < visible; ++j)
				if ((idx = uni(rng) * j) < cnt)
					vis[idx] = vis[j];
		}
		vis.resize(std::min(visible, cnt));

//		auto p = rfs->createFeaturePoint();
//		p->reprojectedPosition =  p->position = pt;
//		p->color = corecvs::RGBColor(0, 0, 255);
		std::unordered_map<WPP, int> idx;
		for (auto wpp: vis)
		{
			auto proj = wpp.u->project(pt, wpp.v) + Vector2dd(rnorm(rng), rnorm(rng));
			idx[wpp] = rfs->keyPoints[wpp].size();
			rfs->keyPoints[wpp].emplace_back(proj, corecvs::RGBColor(0, 0, 255));
		}
		for (auto wpp1: vis)
		{
			for (auto wpp2: vis)
			{
				if (wpp1 == wpp2)
					continue;
				if (!(wpp1 < wpp2))
					continue;
				rfs->matches[wpp1][wpp2].emplace_back(idx[wpp1], idx[wpp2], 0.5);
			}
		}
	}

}

CameraModel SceneGenerator::generateCamera(int id)
{
	double w = 4000.0, h = 3000.0, phi = 70.0, psi = 60.0, r = 0.12;
	CameraModel cm;
	cm.intrinsics.size      = Vector2dd(w, h);
	cm.intrinsics.principal = Vector2dd(w / 2.0, h / 2.0);
	double f = w / 2.0 / tan(phi / 2.0 / 180.0 * M_PI);
	cm.intrinsics.focal     = Vector2dd(f, f);

	cm.extrinsics.position = Vector3dd(cos(id * psi / 180.0 * M_PI), sin(id * psi / 180.0 * M_PI), 0.0) * r;
	cm.extrinsics.orientation = Quaternion(0, sin(id * psi / 180.0 * M_PI / 2.0), 0, cos(id * psi / 180.0 * M_PI / 2.0)) ^ Quaternion::FromMatrix(Matrix33(0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0));
	cm.nameId = std::to_string(id);
	return cm;
}

CameraFixture* SceneGenerator::generatePs(corecvs::Vector3dd pos, int id)
{
	std::string tag = "";
	do
	{
		tag = (char)('A' + (id % 26)) + tag;
		id /= 26;
	} while (id > 0);

	auto f = rfs->createCameraFixture();
	for (int i = 0; i < 6; ++i)
	{
		auto c = rfs->createCamera();
		auto cc = generateCamera(i);
		c->intrinsics = cc.intrinsics;
		c->extrinsics = cc.extrinsics;
		c->nameId = cc.nameId;
		rfs->addCameraToFixture(c, f);
	}
	f->location.shift = pos;
	rfs->initializationData[f].initData = f->location;
	rfs->initializationData[f].initializationType = PhotostationInitializationType::GPS;
	return f;
}

