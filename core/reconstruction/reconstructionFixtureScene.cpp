#include "reconstructionFixtureScene.h"

#include <set>
#include <unordered_set>

#include "featureMatchingPipeline.h"
#include "bufferReaderProvider.h"
#include "abstractPainter.h"
#include "multicameraTriangulator.h"
#include "essentialFeatureFilter.h"
#include "log.h"

using namespace corecvs;

ReconstructionFixtureScene::ReconstructionFixtureScene()
{
}


void ReconstructionFixtureScene::transform(const corecvs::Affine3DQ &transform, double scale)
{
    std::cout << "RFS::transform::parent" << std::endl;
    FixtureScene::transform(transform, scale);
    std::cout << "RFS::transform::points" << std::endl;
    corecvs::parallelable_for(0, (int)trackedFeatures.size(), [&](const corecvs::BlockedRange<int> &r) { for (int i = r.begin(); i < r.end(); ++i)
            { auto& pt = trackedFeatures[i];
              pt->reprojectedPosition = pt->triangulate(true);
              }
              });
    std::cout << "RFS::transform::finished" << std::endl;
}

void ReconstructionFixtureScene::printPosStats()
{
    for (auto ptr: placedFixtures)
        std::cout << ptr->name << " " << ptr->location.shift << ptr->location.rotor << std::endl;
    std::cout << "\t";
    for (auto ptrA: placedFixtures)
        std::cout << ptrA->name << "\t";
    std::cout << std::endl;
    for (auto ptrA: placedFixtures)
    {
        std::cout << ptrA->name << "\t";
        for (auto ptrB: placedFixtures)
            std::cout << !(ptrA->location.shift - ptrB->location.shift) << "\t";
        std::cout << std::endl;
    }
}

void ReconstructionFixtureScene::printMatchStats()
{
    std::vector<CameraFixture*> fix;
    for (auto ptr: fixtures()) fix.push_back(ptr);

    std::sort(fix.begin(), fix.end(), [](CameraFixture* a, CameraFixture* b) { return a->name < b->name; });

    for (size_t i1 = 0; i1 < fix.size(); ++i1)
    {
        auto f1 = fix[i1];
        std::vector<FixtureCamera*> cams1;
        for (auto c: f1->cameras) cams1.push_back(c);
        std::sort(cams1.begin(), cams1.end(), [](FixtureCamera* a, FixtureCamera* b) { return a->nameId < b->nameId; });

        for (size_t i2 = i1; i2 < fix.size(); ++i2)
        {
            auto f2 = fix[i2];
            std::vector<FixtureCamera*> cams2;
            for (auto c: f2->cameras) cams2.push_back(c);
            std::sort(cams2.begin(), cams2.end(), [](FixtureCamera* a, FixtureCamera* b) { return a->nameId < b->nameId; });

            std::cout << f1->name << " x " << f2->name << std::endl << "\t";
            for (auto c: cams2)
                std::cout << c->nameId << "\t";
            std::cout << std::endl;
            for (auto c1: cams1)
            {
                std::cout << c1->nameId << "\t";
                for (auto c2: cams2)
                {
                    WPP id1(f1, c1), id2(f2, c2);
                    std::cout << (matches.count(id1) && matches[id1].count(id2) ?
                                    matches[id1][id2].size() :
                                    matches.count(id2) && matches[id2].count(id1) ? matches[id2][id1].size() : 0
                                 ) << "\t";
                }
                std::cout << std::endl;
            }
        }
    }
}

void ReconstructionFixtureScene::printTrackStats()
{
    std::unordered_map<std::pair<CameraFixture*, CameraFixture*>, int> cntr;
    for (auto& pt: trackedFeatures)
        for (auto& o1: pt->observations__)
            for (auto &o2: pt->observations__)
                ++cntr[std::make_pair(o1.second.cameraFixture, o2.second.cameraFixture)];
    std::cout << "\t";
    for (auto& cf: placedFixtures)
        std::cout << cf->name << "\t";
    std::cout << std::endl;
    for (auto& A: placedFixtures)
    {
        std::cout << A->name << "\t";
        for (auto& B: placedFixtures)
        {
            std::cout << cntr[std::make_pair(A, B)] << "\t";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

    std::vector<double> errs, ssqs;
    for (auto& pt: trackedFeatures)
    {
        double ssq = 0.0, cnt = 0.0;
        for (auto& o: pt->observations__)
        {
            auto err = !(o.first.u->reprojectionError(pt->reprojectedPosition, o.second.observation, o.first.v));
            errs.push_back(err * err);
            ssq += err * err;
            cnt += 1.0;
        }
        ssqs.push_back(std::sqrt(ssq / cnt));
    }
    double tolerance = 0.1;
    std::map<int, int> cntE, cntS;
    for (auto& e: errs)
        ++cntE[e / tolerance];
    for (auto& s: ssqs)
        ++cntS[s / tolerance];
    std::cout << "Reprojection histogram: " << std::endl;
    for (int i = 0; i <= cntE.rbegin()->first; ++i)
        std::cout << "[" << tolerance * i << "; " << tolerance * (i + 1) << ") : " << cntE[i] << std::endl;
    std::cout << "RMSE histogram: " << std::endl;
    for (int i = 0; i <= cntS.rbegin()->first; ++i)
        std::cout << "[" << tolerance * i << "; " << tolerance * (i + 1) << ") : " << cntS[i] << std::endl;

}

FixtureScene* ReconstructionFixtureScene::dumbify()
{
    auto dumb = new FixtureScene();
    std::unordered_map<WPP, WPP> wppmap;
    for (auto f: fixtures())
    {
        auto ff = dumb->createCameraFixture();
        ff->location = f->location;
        ff->name = f->name;
        for (auto c: f->cameras)
        {
            auto cc = dumb->createCamera();
            cc->intrinsics = c->intrinsics;
            cc->extrinsics = c->extrinsics;
            cc->distortion = c->distortion;
            cc->nameId     = c->nameId;
            dumb->addCameraToFixture(cc, ff);
            wppmap[WPP(f, c)] = WPP(ff, cc);
        }
    }
    for (auto p : featurePoints())
    {
        auto pp = dumb->createFeaturePoint();
        pp->reprojectedPosition = p->reprojectedPosition;
        pp->color = p->color;
        for (auto &o: p->observations__)
        {
            auto& oo = pp->observations[wppmap[o.first].v] = o.second;
            auto wpp = wppmap[o.first];
            oo.camera = wpp.v;
            oo.cameraFixture = wpp.u;
            oo.featurePoint = pp;
        }
    }
    return dumb;
}

void ReconstructionFixtureScene::pruneTracks(double threshold)
{
    int id = 0;
    for (auto& pt: trackedFeatures)
    {
        auto pos = pt->reprojectedPosition;
        double ssq = 0.0, cnt = 0.0, mxe = 0.0;
        for (auto& obs: pt->observations__)
        {
            auto err = obs.first.u->reprojectionError(pos, obs.second.observation, obs.first.v);
            ssq += !err * !err;
            mxe = std::max(!err, mxe);
            cnt += 1.0;
        }
        if ((ssq / cnt) < threshold * threshold && mxe < threshold * 2.0)
        {
            trackedFeatures[id++] = pt;
        }
        else
        {
            std::vector<std::pair<WPP, int>> removal;
            for (auto& m: trackMap)
                for (auto& md: m.second)
                    if (md.second == pt)
                        removal.emplace_back(m.first, md.first);
            for (auto& d: removal)
                trackMap[d.first].erase(d.second);
        }
    }
    int pruned = trackedFeatures.size() - id;
    double ratio = double(pruned) / trackedFeatures.size() * 100.0;
    std::cout << pruned << " tracks (" << ratio << ") were pruned!" << std::endl;
    trackedFeatures.resize(id);
}

void ReconstructionFixtureScene::deleteCamera(FixtureCamera *camera)
{
    FixtureScene::deleteCamera(camera);
    deleteFixtureCameraUMWPP(images,    camera);
    deleteFixtureCameraUMWPP(keyPoints, camera);
    deleteFixtureCameraUMWPP(matches,   camera);
    deleteFixtureCameraUMWPP(trackMap,  camera);
}

void ReconstructionFixtureScene::deleteCameraFixture(CameraFixture *fixture, bool recursive)
{
    std::cout << "deleting from parent" << std::endl;
    FixtureScene::deleteCameraFixture(fixture, recursive);
    std::cout << "deleting from images" << std::endl;
    deleteCameraFixtureUMWPP(images,    fixture);
    std::cout << "deleting from keypoints" << std::endl;
    deleteCameraFixtureUMWPP(keyPoints, fixture);
    std::cout << "deleting from matches" << std::endl;
    deleteCameraFixtureUMWPP(matches,   fixture);
    std::cout << "deleting from trackmap" << std::endl;
    deleteCameraFixtureUMWPP(trackMap,  fixture);
    std::cout << "deleting from init" << std::endl;
    initializationData.erase(fixture);
    std::cout << "deleting from placed" << std::endl;
    vectorErase(placedFixtures, fixture);
    std::cout << "deleting from queue" << std::endl;
    vectorErase(placingQueue, fixture);
}

void ReconstructionFixtureScene::deleteFixturePair(CameraFixture *fixture, FixtureCamera *camera)
{
    FixtureScene::deleteFixturePair(fixture, camera);
    deletePairUMWPP(images,    fixture, camera);
    deletePairUMWPP(keyPoints, fixture, camera);
    deletePairUMWPP(matches,   fixture, camera);
    deletePairUMWPP(trackMap,  fixture, camera);
}

void ReconstructionFixtureScene::deleteFeaturePoint(SceneFeaturePoint *point)
{
    FixtureScene::deleteFeaturePoint(point);
    vectorErase(trackedFeatures, point);
    vectorErase(staticPoints, point);
    for (auto& foo: trackMap)
        for (auto boo = foo.second.begin(); boo != foo.second.end(); boo = boo->second == point ? foo.second.erase(boo) : boo++);
}

void ReconstructionFixtureScene::detectAllFeatures(const FeatureDetectionParams &params)
{
    // Mapping from indexes to fixture,camera pairs
    std::unordered_map<int, WPP> map;
    std::vector<std::string> filenames;
    for (auto& f: placingQueue)
    {
        for (auto& c: f->cameras)
        {
            WPP idx(f, c);
            if (!images.count(idx))
                continue;
            filenames.push_back(images[idx]);
            map[(int)filenames.size() - 1] = idx;
            CORE_ASSERT_TRUE_S(idx.u != WPP::UWILDCARD && idx.v != WPP::VWILDCARD);
        }
    }
    size_t N = filenames.size();
    // Cleanup
    for (auto& pp: map)
    {
        keyPoints[pp.second].clear();
        matches[pp.second].clear();
        for (auto& id: matches)
        {
            if (id.second.count(pp.second))
            {
                id.second[pp.second].clear();
            }
        }
        trackMap[pp.second].clear();
    }

    // Feature detection and matching
    FeatureMatchingPipeline pipeline(filenames);
    pipeline.add(new KeyPointDetectionStage(params.detector), true);
    pipeline.add(new DescriptorExtractionStage(params.descriptor), true);
    pipeline.add(new MatchingPlanComputationStage(), true);
    pipeline.add(new MatchAndRefineStage(params.descriptor, params.matcher, params.b2bThreshold), true);

    pipeline.run();

    // Here we discard some data and store only part (w/o descriptors and etc)
    for (size_t i = 0; i < N; ++i)
    {
        auto& kps = pipeline.images[i].keyPoints.keyPoints;
        CORE_ASSERT_TRUE_S(map.count((int)i));
        auto id = map[(int)i];

        auto& kpp = keyPoints[id];
        for (auto& kp: kps)
        {
            kpp.emplace_back(Vector2dd(kp.x, kp.y), kp.color);
        }
    }
    // Since our matches are reflective, we can store them only once
    /*
     * XXX: I've added flag for merging matches from all cameras.
     *         This works almost as if we were merging all pictures from
     *         all cameras of photostation into a single image
     */
    if (!params.matchF2F)
    {
        auto& ref = pipeline.refinedMatches.matchSets;
        for (auto& ms: ref)
        {
            CORE_ASSERT_TRUE_S(map.count((int)ms.imgA));
            CORE_ASSERT_TRUE_S(map.count((int)ms.imgB));
            auto id1 = map[(int)ms.imgA];
            auto id2 = map[(int)ms.imgB];
            bool swap = !(id1 < id2);

            auto idA = swap ? id2 : id1;
            auto idB = swap ? id1 : id2;
            CORE_ASSERT_TRUE_S(idA.u != WPP::UWILDCARD && idA.v != WPP::VWILDCARD);
            CORE_ASSERT_TRUE_S(idB.u != WPP::UWILDCARD && idB.v != WPP::VWILDCARD);

            auto& mset = matches[idA][idB];
            for (auto& m: ms.matches)
            {
                int fA = swap ? m.featureB : m.featureA;
                int fB = swap ? m.featureA : m.featureB;
                mset.emplace_back(fA, fB, m.best2ndBest);
            }
        }
    }
    else
    {
        auto &ref = pipeline.refinedMatches.matchSets;
        std::unordered_map<std::pair<WPP, int>, std::vector<std::tuple<WPP, int, double, double>>> matchMap;
        for (auto& ms: ref)
        {
            CORE_ASSERT_TRUE_S(map.count((int)ms.imgA));
            CORE_ASSERT_TRUE_S(map.count((int)ms.imgB));
            auto id1 = map[(int)ms.imgA];
            auto id2 = map[(int)ms.imgB];
            bool swap = !(id1 < id2);

            auto idA = swap ? id2 : id1;
            auto idB = swap ? id1 : id2;
            CORE_ASSERT_TRUE_S(idA.u != WPP::UWILDCARD && idA.v != WPP::VWILDCARD);
            CORE_ASSERT_TRUE_S(idB.u != WPP::UWILDCARD && idB.v != WPP::VWILDCARD);

#if 0
            auto& mset = matches[idA][idB];
            for (auto& m: ms.matches)
            {
                int fA = swap ? m.featureB : m.featureA;
                int fB = swap ? m.featureA : m.featureB;
                mset.emplace_back(fA, fB, m.best2ndBest);
            }
#else
            for (auto& m: ms.matches)
            {
                int fA = swap ? m.featureB : m.featureA;
                int fB = swap ? m.featureA : m.featureB;
                matchMap[std::make_pair(idA, fA)].emplace_back(idB, fB, m.distance, m.best2ndBest);
            }

#endif
        }
        for (auto& p: matchMap)
        {
            auto& vp = p.second;
            std::sort(vp.begin(), vp.end(), [&](const decltype(vp[0]) &a, const decltype(vp[0]) &b) { return std::get<2>(a) < std::get<2>(b); });
            if (vp.size() > 1)
            {
                double dA1 = std::get<2>(vp[0]),
                       dB1 = std::get<2>(vp[1]);
                double dA2 = dA1 / std::get<3>(vp[0]),
                       dB2 = dB1 / std::get<3>(vp[1]);
                double best2ndBest = dA1 / std::max(dA2, std::max(dB1, dB2));
                std::get<3>(vp[0]) = best2ndBest;
                vp.resize(1);
            }
        }
        for (auto& vp: matchMap)
        {
            if (!(vp.first.first < std::get<0>(vp.second[0])))
                continue;
            auto& v = matchMap[std::make_pair(std::get<0>(vp.second[0]), std::get<1>(vp.second[0]))];
            if (!v.size())
                continue;
            auto wpp = std::get<0>(v[0]);
            auto fea = std::get<1>(v[0]);
            if (!(wpp == vp.first.first))
                continue;
            if (fea != vp.first.second)
                continue;
            matchMap[std::make_pair(vp.first.first, vp.first.second)].emplace_back(wpp, std::get<1>(vp.second[0]), std::get<2>(vp.second[0]), std::get<3>(vp.second[0]));
        }
    }
    printMatchStats();
}

std::vector<FixtureCamera*> ReconstructionFixtureScene::getDistinctCameras() const
{
    std::cout << "GDC" << std::endl;
    std::set<FixtureCamera*> cameras;
    for (auto& f: placedFixtures)
    {
        for (auto& c: f->cameras)
        {
            cameras.insert(c);
        }
    }
    std::cout << cameras.size() << std::endl;
    return std::vector<FixtureCamera*>(cameras.begin(), cameras.end());
}

int ReconstructionFixtureScene::getDistinctCameraCount() const
{
    return (int)getDistinctCameras().size();
}

bool ReconstructionFixtureScene::validateMatches()
{
    for (auto& mv: matches)
    {
        auto fixture = mv.first.u;
        auto camera  = mv.first.v;
        CORE_ASSERT_TRUE_S(std::find(fixtures().begin(), fixtures().end(), fixture) != fixtures().end());
//        CORE_ASSERT_TRUE_S(std::find(cameras.begin(), cameras.end(), cameras) != cameras.end());

        for (auto& mvv: mv.second)
        {
            auto fixtureB = mvv.first.u;
            auto cameraB  = mvv.first.v;
            CORE_ASSERT_TRUE_S(std::find(fixtures().begin(), fixtures().end(), fixtureB) != fixtures().end());
  //          CORE_ASSERT_TRUE_S(std::find(cameras.begin(), cameras.end(), cameraB) != cameras.end());
            for (auto& t: mvv.second)
            {
                auto mA = std::get<0>(t),
                     mB = std::get<1>(t);
                auto itA = keyPoints.count(WPP(fixture, camera));
                CORE_ASSERT_TRUE_S((int)itA > 0);
                CORE_ASSERT_TRUE_S(keyPoints[WPP(fixture, camera)].size() > (size_t)mA);
                auto itB = keyPoints.count(WPP(fixtureB, cameraB));
                CORE_ASSERT_TRUE_S((int)itB > 0);
                CORE_ASSERT_TRUE_S(keyPoints[WPP(fixtureB, cameraB)].size() > (size_t)mB);
            }
        }
    }
    return true;
}

bool ReconstructionFixtureScene::haveCamera(FixtureCamera * camera)
{
    for (auto& p: mOwnedObjects)
    {
        if (dynamic_cast<FixtureCamera*>(p) == camera)
            return true;
    }
    return false;
}

bool ReconstructionFixtureScene::haveFixture(CameraFixture * camera)
{
    for (auto& p: mOwnedObjects)
    {
        if (dynamic_cast<CameraFixture*>(p) == camera)
            return true;
    }
    return false;
}

bool ReconstructionFixtureScene::havePoint(SceneFeaturePoint *point)
{
    for (auto& p: mOwnedObjects)
    {
        if (dynamic_cast<SceneFeaturePoint*>(p) == point)
            return true;
    }
    return false;
}

bool ReconstructionFixtureScene::validateTracks()
{
    for (auto pt: trackedFeatures)
    {
        CORE_ASSERT_TRUE_S(pt);
        CORE_ASSERT_TRUE_S(std::find(featurePoints().begin(), featurePoints().end(), pt) != featurePoints().end());
        for (auto wpp: pt->observations__)
        {
            auto fixture = wpp.first.u;
            auto camera  = wpp.first.v;
            CORE_ASSERT_TRUE_S(std::find(fixtures().begin(), fixtures().end(), fixture) != fixtures().end());
            auto o       = wpp.second;
            CORE_ASSERT_TRUE_S(std::find(fixture->cameras.begin(), fixture->cameras.end(), camera) != fixture->cameras.end());
            CORE_ASSERT_TRUE_S(fixture == o.cameraFixture);
            CORE_ASSERT_TRUE_S(camera  == o.camera);
            CORE_ASSERT_TRUE_S(pt      == o.featurePoint);
            for (int i = 0; i < 2; ++i)
                CORE_ASSERT_TRUE_S(!std::isnan(o.observation[i]));
        }
        for (int i = 0; i < 3; ++i)
            CORE_ASSERT_TRUE_S(!std::isnan(pt->reprojectedPosition[i]));
    }
    return true;
}

bool ReconstructionFixtureScene::validateAll()
{
    L_INFO << "Validating...";
    if (validateMatches() && validateTracks())
    {
        L_INFO << "VALID!!!";
        return true;
    }
    return false;
}

void ParallelTrackPainter::operator() (const corecvs::BlockedRange<int> &r) const
{
    if (!pairs)
    {
        for (int i = r.begin(); i < r.end(); ++i)
        {
            auto& p = images[i];
            auto key = p.first;
            auto name= p.second;
            std::stringstream ss;
            ss << name << "_tracks.png";

            auto nameNew = ss.str();
            corecvs::RGB24Buffer src = BufferReaderProvider::readRgb(name);

            AbstractPainter<RGB24Buffer> painter(&src);
            for (auto& tf: scene->trackedFeatures)
            {
                for (auto& obs: tf->observations__)
                    if (obs.first == key)
                    {
                        painter.drawFormat(obs.second.observation[0] + 5, obs.second.observation[1], colorizer[0][tf], 1,  tf->name.c_str());
                        painter.drawCircle(obs.second.observation[0], obs.second.observation[1], 3, colorizer[0][tf]);
                    }
            }
            BufferReaderProvider::writeRgb(src, nameNew);
            std::cout << "Writing tracks image into " << nameNew << std::endl;
        }
    }
    else
    {
        for (int ii = r.begin(); ii < r.end(); ++ii)
        {
            int i = ii % (int)images.size(),
                j = ii / (int)images.size();
            if (i >= j)
                continue;

            auto& imgA = images[i],
                & imgB = images[j];
            auto& keyA = imgA.first,
                & keyB = imgB.first;
            auto& nameA= imgA.second,
                & nameB= imgB.second;
            std::stringstream ss;
            ss << keyA.u->name << keyA.v->nameId << keyB.u->name << keyB.v->nameId << "_tracks.jpg";

            auto srcA = BufferReaderProvider::readRgb(nameA),
                 srcB = BufferReaderProvider::readRgb(nameB);

            int newW = std::max(srcA.w, srcB.w);
            int newH = srcA.h + srcB.h;
            int offH = srcA.h;
            corecvs::RGB24Buffer dst(newH, newW);
            AbstractPainter<RGB24Buffer> painter(&dst);

            for (int y = 0; y < srcA.h; ++y)
                for (int x = 0; x < srcA.w; ++x)
                    dst.element(y, x) = srcA.element(y, x);
            for (int y = 0; y < srcB.h; ++y)
                for (int x = 0; x < srcB.w; ++x)
                    dst.element(y + offH, x) = srcB.element(y, x);

            bool painted = false;
            for (auto& tf: scene->trackedFeatures)
            {
                std::remove_reference<decltype(tf->observations__[0])>::type* obsA = 0, *obsB = 0;
                for (auto& obs: tf->observations__)
                {
                    if (obs.first == keyA)
                        obsA = &obs.second;
                    if (obs.first == keyB)
                        obsB = &obs.second;
                }
                if (!obsA || !obsB)
                    continue;

                painter.drawFormat(obsA->observation[0] + 5, obsA->observation[1], colorizer[0][tf], 1,  tf->name.c_str());
                painter.drawCircle(obsA->observation[0], obsA->observation[1], 3, colorizer[0][tf]);
                painter.drawFormat(obsB->observation[0] + 5, obsB->observation[1] + offH, colorizer[0][tf], 1,  tf->name.c_str());
                painter.drawCircle(obsB->observation[0], obsB->observation[1] + offH, 3, colorizer[0][tf]);
                dst.drawLine(
                        obsB->observation[0],
                        obsB->observation[1] + offH,
                        obsA->observation[0],
                        obsA->observation[1],
                        colorizer[0][tf]);
                painted = true;
            }
            if (!painted)
                continue;

            BufferReaderProvider::writeRgb(dst, ss.str());
            std::cout << "Written to " << ss.str() << std::endl;
        }
    }
}

std::vector<std::tuple<FixtureCamera*, corecvs::Vector2dd, corecvs::Vector3dd, SceneFeaturePoint*, int>> corecvs::ReconstructionFixtureScene::getPossibleTracks(CameraFixture *psA)
{
    CORE_ASSERT_TRUE_S(state == ReconstructionState::APPENDABLE || state == ReconstructionState::TWOPOINTCLOUD || state == ReconstructionState::FINISHED);

    std::unordered_set<SceneFeaturePoint*> selectedTracks;
    std::vector<std::tuple<FixtureCamera*, corecvs::Vector2dd, corecvs::Vector3dd, SceneFeaturePoint*, int>> res;

    for (auto& psB: placedFixtures)
    {
        for (auto& camB: psB->cameras)
        {
            for (auto& camA: psA->cameras)
            {
                WPP idB(psB, camB), idA(psA, camA);
                auto& kPoints = keyPoints[idA];
                bool swap = idB < idA;
                auto id1 = swap ? idB : idA;
                auto id2 = swap ? idA : idB;

                if (!matches.count(id1))
                    continue;
                auto& mmv = matches[id1];
                if (!mmv.count(id2))
                    continue;

                auto& mm = mmv[id2];
                for (auto& m: mm)
                {
                    int pt1 = std::get<0>(m),
                        pt2 = std::get<1>(m);
                    int ptA = swap ? pt2 : pt1;
                    int ptB = swap ? pt1 : pt2;
                    if (!trackMap.count(idB) ||
                        !trackMap[idB].count(ptB))
                        continue;
                    auto track = trackMap[idB][ptB];
                    if (selectedTracks.count(track))
                        continue;
                    res.emplace_back(
                        camA,
                        kPoints[ptA].first,
                        track->reprojectedPosition,
                        track,
                        ptA);

                    selectedTracks.insert(track);
                }
            }
        }
    }
    return res;
}

void corecvs::ReconstructionFixtureScene::buildTracks(CameraFixture *psA, CameraFixture *psB, double trackInlierThreshold, double distanceLimit)
{
    const int NPS = 2;
    const int NPAIRS = 1;
    int pairIdx[NPAIRS][2] = {{0, 1}};

    CameraFixture*        ps[NPS] = {psA, psB};
    FixtureCamera*       cam[NPS] = {  0,   0};
    int                   pt[NPS] = {  0,   0};
    FixtureCamera* &camA = cam[0], *&camB = cam[1];
    int &ptA = pt[0], &ptB = pt[1];
    WPP                  wpp[NPS];
    corecvs::Vector2dd    kp[NPS];
  //corecvs::Vector2dd &kpA = kp[0], &kpB = kp[1], &kpC = kp[2];

    std::unordered_map<std::tuple<FixtureCamera*, FixtureCamera*, int>, int> free[NPAIRS];
    auto &freeAB = free[0];

    for (int i = 0; i < NPAIRS; ++i)
        free[i] = getUnusedFeatures(ps[pairIdx[i][0]], ps[pairIdx[i][1]]);

    std::vector<std::tuple<FixtureCamera*, int, FixtureCamera*, int>> trackCandidates;

    for (auto& mAB: free[0])
    {
        camA = std::get<0>(mAB.first);
        camB = std::get<1>(mAB.first);
        ptA = std::get<2>(mAB.first);
        ptB = mAB.second;

        trackCandidates.emplace_back(camA, ptA, camB, ptB);
    }
    L_INFO << trackCandidates.size() << " candidate tracks";
    L_INFO << "Inlier threshold: " << trackInlierThreshold;
    L_INFO << "Distance threshold: " << distanceLimit;

    int failInlier = 0, failDistance = 0;

    for (auto& c: trackCandidates)
    {
        camA = std::get<0>(c);
        camB = std::get<2>(c);
        ptA = std::get<1>(c);
        ptB = std::get<3>(c);

        bool alreadyIn = false;
        for(int i = 0; i < NPS; ++i)
        {
            wpp[i] = WPP(ps[i], cam[i]);
            kp[i] = keyPoints[wpp[i]][pt[i]].first;
            if (trackMap[wpp[i]].count(pt[i]))
                alreadyIn = true;
        }
        if (alreadyIn)
            continue;

        double fscore = 0.0;
        for (int i = 0; i < NPAIRS; ++i)
        {
            int id1 = pairIdx[i][0];
            int id2 = pairIdx[i][1];
            fscore = std::max(fscore, ps[id1]->scoreFundamental(cam[id1], kp[id1], ps[id2], cam[id2], kp[id2]));
        }
        if (fscore > trackInlierThreshold)
        {
            failInlier++;
            continue;
        }

        corecvs::MulticameraTriangulator mct;
        for (int i = 0; i < NPS; ++i)
            mct.addCamera(ps[i]->getMMatrix(cam[i]), kp[i]);
        auto res = mct.triangulateLM(mct.triangulate());

        bool isVisibleInlierNotTooFar = true;
        for (int i = 0; i < NPS; ++i)
        {
            isVisibleInlierNotTooFar &= ps[i]->isVisible(res, cam[i]);
            isVisibleInlierNotTooFar &= (!(kp[i] - ps[i]->project(res, cam[i]))) < trackInlierThreshold;
            isVisibleInlierNotTooFar &= (!(res - ps[i]->getWorldCamera(cam[i]).extrinsics.position)) < distanceLimit;
        }
        if (!isVisibleInlierNotTooFar)
        {
            failDistance++;
            continue;
        }

        auto track = createFeaturePoint();
        track->reprojectedPosition = res;
        track->hasKnownPosition = false;
        track->type = SceneFeaturePoint::POINT_RECONSTRUCTED;

        for (int i = 0; i < NPS; ++i)
        {
            SceneObservation so;
            so.camera = cam[i];
            so.cameraFixture = ps[i];
            so.featurePoint = track;
            so.observation = kp[i];
            track->observations[cam[i]] = so;
            track->observations__[wpp[i]] = so;
            track->color = keyPoints[wpp[i]][pt[i]].second;
            trackMap[wpp[i]][pt[i]] = track;
        }
        trackedFeatures.push_back(track);
    }

    L_INFO << "FAIL:IT " << failInlier << " / FAIL:DI " << failDistance;
}

std::unordered_map<std::tuple<FixtureCamera*, FixtureCamera*, int>, int> corecvs::ReconstructionFixtureScene::getUnusedFeatures(CameraFixture *psA, CameraFixture *psB)
{
    std::unordered_map<std::tuple<FixtureCamera*, FixtureCamera*, int>, int> res;
    for (auto camA: psA->cameras)
    {
        for (auto camB: psB->cameras)
        {
            WPP idA(psA, camA), idB(psB, camB);
            bool swap = !(idA < idB);
            auto id1 = swap ? idB : idA, id2 = swap ? idA : idB;
            if (!matches.count(id1))
                continue;
            if (!matches[id1].count(id2))
                continue;
            auto& mv = matches[id1][id2];
            for (auto &m: mv)
            {
                int f1 = std::get<0>(m),
                    f2 = std::get<1>(m);
                if (trackMap[id1].count(f1))
                    continue;
                if (trackMap[id2].count(f2))
                    continue;
                int fA = swap ? f2 : f1,
                    fB = swap ? f1 : f2;
                res[std::make_tuple(camA, camB, fA)] = fB;
            }
        }
    }
    return res;
}

void corecvs::ReconstructionFixtureScene::appendTracks(CameraFixture *ps, double trackInlierThreshold, double distanceLimit)
{
    auto candidates = getFixtureMatchesIdx(placedFixtures, ps);
    std::cout << "AP-CAND: " << candidates.size() << std::endl;
    // Now rebuild in order to select only the best appendable for every kp in ps
    std::unordered_map<std::pair<WPP, int>, std::set<corecvs::SceneFeaturePoint*>> mapper;
    int tt = 0, qq = 0;
    for (auto& t: candidates)
    {
        auto idQuery = std::get<2>(t);
        auto ptQuery = std::get<3>(t);
        CORE_ASSERT_TRUE_S(idQuery.u == ps);
        auto idTrain = std::get<0>(t);
        auto ptTrain = std::get<1>(t);
        if (!trackMap[idTrain].count(ptTrain))
        {
            tt++;
            continue;
        }
        if ( trackMap[idQuery].count(ptQuery))
        {
            qq++;
            continue;
        }

        auto track = trackMap[idTrain][ptTrain];
        mapper[std::make_pair(idQuery, ptQuery)].insert(track);
    }
    std::cout << "AP-MAP: " << mapper.size() << std::endl;
    std::cout << "AP-MAP failures: train is not mapped: " << tt << " query is mapped: " << qq << std::endl;
    size_t cnt = mapper.size(), app = 0;
    // Then select best-fitting track and merge 'em until error is OK
    for (auto& pat: mapper)
    {
        SceneFeaturePoint* best = nullptr;
        double bestScore = trackInlierThreshold;
        auto qq = pat.first.first;
        auto pq = pat.first.second;
        auto p  = keyPoints[qq][pq].first;
        auto vv = qq.u->getWorldCamera(qq.v).extrinsics.position;
        for (auto& track: pat.second)
        {
            if ((!(vv - track->reprojectedPosition)) > distanceLimit)
                continue;
            if (!qq.u->isVisible(track->reprojectedPosition, qq.v))
                continue;
            double err = -1.0;
            if ((err = !(qq.u->project(track->reprojectedPosition, qq.v) - p)) >= trackInlierThreshold)
                continue;
            if (err < bestScore)
            {
                bestScore = err;
                best = track;
            }
        }
        if (!best)
            continue;
        SceneObservation so;
        so.camera = qq.v;
        so.cameraFixture = qq.u;
        so.featurePoint = best;
        so.observation = p;
        best->observations[so.camera] = so;
        best->observations__[qq] = so;
        trackMap[qq][pq] = best;
        best->reprojectedPosition = best->triangulate();
        ++app;
    }
    std::cout << "TA: (" << ps->name << ")"  << cnt << " / " << app << std::endl;
}

std::vector<std::tuple<WPP, corecvs::Vector2dd, WPP, corecvs::Vector2dd, double>>
corecvs::ReconstructionFixtureScene::getFixtureMatches(const std::vector<CameraFixture*> &train, CameraFixture *query)
{
    std::vector<std::tuple<WPP, corecvs::Vector2dd, WPP, corecvs::Vector2dd, double>> res;
    auto idxs = getFixtureMatchesIdx(train, query);
    for (auto& it: idxs)
    {
        auto &kpsA = keyPoints[std::get<0>(it)],
             &kpsB = keyPoints[std::get<2>(it)];
        res.emplace_back(std::get<0>(it), kpsA[std::get<1>(it)].first, std::get<2>(it), kpsB[std::get<3>(it)].first, std::get<4>(it));
    }
    return res;
}

std::vector<std::tuple<WPP, int, WPP, int, double>>
corecvs::ReconstructionFixtureScene::getFixtureMatchesIdx(const std::vector<CameraFixture*> &train, CameraFixture *query)
{
    WPP wcQuery = WPP(query, WPP::VWILDCARD), wcTarget;
    std::vector<std::tuple<WPP, int, WPP, int, double>> res;

    for (auto& ref1: matches)
    {
        bool swap = ref1.first == wcQuery;
        if (!swap && train.end() == std::find(train.begin(), train.end(), ref1.first.u))
            continue;

        for (auto& ref2: ref1.second)
        {
            if (!swap && !(ref2.first == wcQuery))
                continue;
            if (swap && train.end() == std::find(train.begin(), train.end(), ref2.first.u))
                continue;

            auto  idA  = swap ? ref2.first : ref1.first;
            auto  idB  = swap ? ref1.first : ref2.first;
            CORE_ASSERT_TRUE_S(idB == wcQuery);
            CORE_ASSERT_TRUE_S(idA.u != WPP::UWILDCARD && idA.v != WPP::VWILDCARD);
            CORE_ASSERT_TRUE_S(idB.u != WPP::UWILDCARD && idB.v != WPP::VWILDCARD);
            auto& kpsA = keyPoints[idA];
            auto& kpsB = keyPoints[idB];
            for (auto& m: ref2.second)
            {
                int kpA = std::get<0>(m);
                int kpB = std::get<1>(m);
                if (swap)
                    std::swap(kpA, kpB);
                res.emplace_back(idA, kpA, idB, kpB, std::get<2>(m));
            }
        }
    }
    return res;
}

void corecvs::ReconstructionFixtureScene::filterEssentialRansac(const std::vector<CameraFixture*> &lhs, const std::vector<CameraFixture*> &rhs, EssentialFilterParams params)
{
    matchesCopy = matches;
    std::vector<std::pair<WPP, WPP>> work;
    for (auto& psA: lhs)
    {
        for (auto& psB: rhs)
        {
            for (auto& camA: psA->cameras)
            {
                for (auto& camB: psB->cameras)
                {
                    WPP idFirst(psA, camA), idSecond(psB, camB);
                    if (psA == psB)
                        continue;
                    bool alreadyIn = false;
                    for (auto& pp: work)
                        if ((pp.first == idFirst && pp.second == idSecond) || (pp.second == idFirst && pp.first == idSecond))
                        {
                            alreadyIn = true;
                            break;
                        }
                    if (!alreadyIn)
                    {
                        work.emplace_back(idFirst, idSecond);
                        if (!essentialCache.count(std::make_pair(idFirst, idSecond)))
                            essentialCache[std::make_pair(idFirst, idSecond)] = std::make_tuple(corecvs::EssentialDecomposition(), 0.0, false);
                    }
                }
            }
        }
    }
    corecvs::parallelable_for(0, (int)work.size(), ParallelEssentialFilter(this, work, params));
}


void corecvs::ReconstructionFixtureScene::remove(WPP a, WPP b, std::vector<int> idx)
{
    bool swap = !(a < b);
    CORE_ASSERT_TRUE_S(matches.count(swap ? b : a));
    CORE_ASSERT_TRUE_S(matches[swap ? b : a].count(swap ? a : b));
    auto& ref = matches[swap ? b : a][swap ? a : b];
    CORE_ASSERT_TRUE_S(idx.size() <= ref.size());
    int ok = 0;
    for (auto& i: idx)
    {
        ref[ok++] = ref[i];
    }
    ref.resize(ok);
}

void corecvs::ReconstructionFixtureScene::filterEssentialRansac(WPP a, WPP b, EssentialFilterParams params)
{
    bool swap = !(a < b);
    WPP idA = swap ? b : a;
    WPP idB = swap ? a : b;

    auto& cache = essentialCache[std::make_pair(idA, idB)];
    bool useCache = std::get<2>(cache);

    std::cout << (!useCache ? "Starting: " : "Using existing estimate for ") << idA.u->name << idA.v->nameId << "<>" << idB.u->name << idB.v->nameId << std::endl;

    std::vector<std::array<corecvs::Vector2dd, 2>> features, featuresInlier;
    auto K1 = idA.v->intrinsics.getKMatrix33();
    auto K2 = idB.v->intrinsics.getKMatrix33();

    if (!matches.count(idA) || !matches[idA].count(idB))
        return;

    auto& mm = matches[idA][idB];
    size_t szBefore = mm.size();
    auto& kpA= keyPoints[idA];
    auto& kpB= keyPoints[idB];
    features.reserve(mm.size());
    featuresInlier.resize(mm.size());

    for (auto& m: mm)
    {
        int idA = std::get<0>(m);
        int idB = std::get<1>(m);
        auto fA = kpA[idA].first;
        auto fB = kpB[idB].first;
        int id = &m - &mm[0];
        featuresInlier[id][0] = fA;
        featuresInlier[id][1] = fB;
        if (std::get<2>(m) < params.b2bThreshold)
           features.push_back(featuresInlier[id]);
    }
    size_t szAfter1= mm.size();
    CORE_ASSERT_TRUE_S(&mm == &matches[idA][idB]);
    CORE_ASSERT_TRUE_S(szBefore == szAfter1);

    EssentialFeatureFilter filter(K1, K2, features, featuresInlier, params);
    double gamma;
    if (!useCache)
    {
        filter.estimate();
        gamma = filter.getGamma();
        auto dec = filter.bestDecomposition;
        std::get<0>(cache) = dec;
        std::get<1>(cache) = gamma;
        std::get<2>(cache) = true;
    }
    else
    {
        gamma = std::get<1>(cache);
        filter.use(std::get<0>(cache));
    }

    auto bestInliers = filter.inlierIdx;

    size_t szAfter = mm.size();
    CORE_ASSERT_TRUE_S(&mm == &matches[idA][idB]);
    CORE_ASSERT_TRUE_S(szBefore == szAfter);
    CORE_ASSERT_TRUE_S(bestInliers.size() <= mm.size());
    std::sort(bestInliers.begin(), bestInliers.end());
    std::cout << "Total: " << featuresInlier.size() << " good: " << features.size() << " del: " << (featuresInlier.size() - bestInliers.size()) << " rem: " << bestInliers.size() << " (" << ((double)bestInliers.size()) / featuresInlier.size() * 100.0 << "%)" << idA.u->name << idA.v->nameId << "<>" << idB.u->name << idB.v->nameId << std::endl;
    if (filter.getGamma() > 0.1)
    {
        std::cout << "Too low gamma-value, rejecting all features" << std::endl;
        bestInliers.clear();
    }
    remove(a, b, bestInliers);
}
