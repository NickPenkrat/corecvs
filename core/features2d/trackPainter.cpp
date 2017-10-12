#include "trackPainter.h"

#include <random>

#include "bufferFactory.h"
#include "abstractPainter.h"

using namespace cvs;

static std::string changeExtension(const std::string &imgName, const std::string &desiredExt)
{
	std::string res(imgName);

	int dotPos = (int)res.size() - 1;
	for (; dotPos >= 0 && res[dotPos] != '.'; --dotPos);
	CORE_ASSERT_TRUE_S(dotPos >= 0);

	res.resize(dotPos + 1 + desiredExt.size());

	std::copy(desiredExt.begin(), desiredExt.end(), res.begin() + dotPos + 1);
	return res;
}

static std::string getFilename(const std::string &imgName)
{
	int pos = (int)imgName.find_last_of(PATH_SEPARATOR[0]);
	std::string res = imgName.substr(pos + 1, imgName.size() - pos);
	return res;
}

static std::string getFiledir(const std::string &imgName)
{
	int pos = (int)imgName.find_last_of(PATH_SEPARATOR[0]);
	std::string res = imgName.substr(0, pos + 1);
	return res;
}

static std::string removeExtension(const std::string &imgName)
{
	int pos = (int)imgName.find_last_of('.');
	std::string res = imgName.substr(0, pos);
	return res;
}

void ParallelTrackPainter::operator() (const corecvs::BlockedRange<int> &r) const
{
    if (!pairs)
    {
        for (int i = r.begin(); i < r.end(); ++i)
        {
            auto& p = images[i];
            auto name = p;
			auto filedir = dir == "" ? getFiledir(name) : dir;
			auto outputName = removeExtension(getFilename(name)) + "_" + suffix + ".jpg";

            corecvs::RGB24Buffer src;
            try {
                src = BufferFactory::getInstance()->readRgb(name);
            }
            catch (...) {
                std::cout << "ParallelTrackPainter:: invalid image " << name << std::endl;
                continue;
            }

            AbstractPainter<RGB24Buffer> painter(&src);
            for (auto& tf : scene)
            {
                RGBColor color = colorizer[tf];
                //painter.drawFormat(obs.second.observation[0] + 5, obs.second.observation[1], color, 1, tf->name.c_str());
                painter.drawCircle((*tf)[0], 3, color);
            }

            auto nameNew = filedir + outputName;
            BufferFactory::getInstance()->saveRGB24Bitmap(src, nameNew);
            std::cout << "Written tracks image to " << nameNew << std::endl;
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
//            auto& keyA = imgA.first,
//                & keyB = imgB.first;
//            auto& nameA= imgA.second,
//                & nameB= imgB.second;
            std::stringstream ss;
			auto filedir = dir == "" ? getFiledir(imgA) : dir;
			auto outputName = removeExtension(getFilename(imgA)) + " " + removeExtension(getFilename(imgB)) + " " + suffix + ".jpg";
			ss << filedir << outputName;

            corecvs::RGB24Buffer srcA, srcB;
            try {
                srcA = BufferFactory::getInstance()->readRgb(imgA);
            }
            catch (...) {
                std::cout << "ParallelTrackPainter:: invalid image " << imgA << std::endl;
                continue;
            }
            try {
                srcB = BufferFactory::getInstance()->readRgb(imgB);
            }
            catch (...) {
                std::cout << "ParallelTrackPainter:: invalid image " << imgB << std::endl;
                continue;
            }
            CORE_ASSERT_TRUE_S(srcA.data && srcB.data);

            int newW = std::max(srcA.w, srcB.w);
            int newH = srcA.h + srcB.h;
            int offH = srcA.h;
            corecvs::RGB24Buffer dst(newH, newW);
            AbstractPainter<RGB24Buffer> painter(&dst);

            //TODO: optimize speed to don't form the not needed image
            // This most probably should be replaced with
            //
            // dst.fillWith(src, 0, 0);
            // dst.fillWith(src, offH, 0);

            for (int y = 0; y < srcA.h; ++y)
                for (int x = 0; x < srcA.w; ++x)
                    dst.element(y, x) = srcA.element(y, x);
            for (int y = 0; y < srcB.h; ++y)
                for (int x = 0; x < srcB.w; ++x)
                    dst.element(y + offH, x) = srcB.element(y, x);

            bool painted = false;
            for (auto& tf : scene)
            {
                auto obsA = (*tf)[0],
                     obsB = (*tf)[1];

                auto a = !obsA;
                auto b = !obsB;
                if (!a || !b)
                    continue;

                RGBColor color = colorizer[tf];
                //painter.drawFormat(obsA->observation[0] + 5, obsA->observation[1]       , color, 1,  tf->name.c_str());
                painter.drawCircle(obsA.x()    , obsA.y()       , 3, color);
                //painter.drawFormat(obsB->observation[0] + 5, obsB->observation[1] + offH, color, 1,  tf->name.c_str());
                painter.drawCircle(obsB.x()    , obsB.y() + offH, 3, color);
                dst    .drawLine  (obsB.x()    , obsB.y() + offH
                                 , obsA.x()    , obsA.y()       ,    color);
                painted = true;
            }
            if (!painted)
                continue;

            BufferFactory::getInstance()->saveRGB24Bitmap(dst, ss.str());
            std::cout << "Written tracks image to " << outputName << std::endl;
        }
    }
}

void TrackPainter::paintTracksOnImages(bool pairs, std::string suffix, std::string outDir)
{
    std::mt19937 rng;
    std::uniform_real_distribution<double> runif(0, 360.0);
    std::unordered_map<Match, RGBColor> colorizer;
    for (auto& tf: matches)
    {
//        std::stringstream ss;
//        ss << tf << ":";
//        size_t cnt = 0;
//        for (auto& o: tf->observations__)
//        {
//            ss << o.first.u->name << o.first.v->nameId;
//            if (++cnt != tf->observations__.size())
//                ss << "/";
//        }
//        tf->name = ss.str();
        colorizer[tf] = corecvs::RGBColor::fromHue(runif(rng), 1.0, 1.0);
    }
//    std::vector<std::pair<WPP, std::string>> images;
//    for (auto& p: scene->images)
//    {
//        bool isPlaced = false;
//        for (auto& cf: scene->placedFixtures)
//            if (cf == p.first.u)
//                isPlaced = true;
//        if (!isPlaced)
//            continue;
//        images.push_back(std::make_pair(p.first, p.second));
//    }
    //cout << "paintTracksOnImages #trackedFeatures=" << matches.size() << " => #images=" << images.size() << endl;

    int N = (int)images.size();
	corecvs::parallelable_for(0, pairs ? N * N : N, ParallelTrackPainter(images, matches, colorizer, pairs, suffix, outDir));

    //cout << "paintTracksOnImages done." << endl;
}
