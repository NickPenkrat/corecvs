#include "featureMatchingPipeline.h"

#include <cassert>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <array>

#include "featureDetectorProvider.h"
#include "descriptorExtractorProvider.h"
#include "descriptorMatcherProvider.h"
#include "bufferReaderProvider.h"
#include "vsfmIo.h"

#ifdef WITH_TBB
#include "tbb/tbb.h"
//#error
#endif

const char* KEYPOINT_EXTENSION = "keypoints";
const char* DESCRIPTOR_EXTENSION = "descriptors";
const char* SIFT_EXTENSION = "sift";

std::string changeExtension(const std::string &imgName, const std::string &desiredExt) {
	std::string res(imgName);

	int dotPos = res.size() - 1;
	for(; dotPos >= 0 && res[dotPos] != '.'; --dotPos);
	assert(dotPos >= 0);

	res.resize(dotPos + 1 + desiredExt.size());

	std::copy(desiredExt.begin(), desiredExt.end(), res.begin() + dotPos + 1);
	return res;
}

FeatureMatchingPipeline::FeatureMatchingPipeline(const std::vector<std::string> &filenames) {
	images.reserve(filenames.size());
	for(size_t i = 0; i < filenames.size(); ++i)
		images.push_back(Image(i,filenames[i]));
}

FeatureMatchingPipeline::~FeatureMatchingPipeline() {
    for(std::vector<FeatureMatchingPipelineStage*>::iterator ps = pipeline.begin(); ps != pipeline.end(); ++ps) {
        (*ps)->~FeatureMatchingPipelineStage();
    }
	pipeline.clear();
}

void FeatureMatchingPipeline::run() {
	for(size_t id = 0; id < pipeline.size(); ++id) {
		auto sParams = saveParams[id];
		auto lParams = loadParams[id];
		auto ps = pipeline[id];

		if(lParams.first)
			ps->loadResults(this, lParams.second);
		if(runEnable[id])
			ps->run(this);
		if(sParams.first)
			ps->saveResults(this, sParams.second);
	}
}

void FeatureMatchingPipeline::add(FeatureMatchingPipelineStage *ps, bool run, std::pair<bool, std::string> saveParams, std::pair<bool, std::string> loadParams) {
	this->saveParams.push_back(saveParams);
	this->loadParams.push_back(loadParams);
	runEnable.push_back(run);
	pipeline.push_back(ps);
}

#ifdef WITH_TBB
class ParallelDetector {
	FeatureMatchingPipeline* pipeline;
	DetectorType detectorType;
public:
	void operator() (const tbb::blocked_range<size_t>& r) const {
		FeatureDetector* detector = FeatureDetectorProvider::getInstance().getDetector(detectorType);
		size_t N = pipeline->images.size();
		size_t id = r.begin();
#if 1
		std::stringstream ss1, ss2;
		ss1 << "Detecting keypoints with " << detectorType << " on ";
		size_t cnt = 0;
		pipeline->tic(id, false);
		size_t kpt = 0;
#endif
		for(size_t i = r.begin(); i != r.end(); ++i) {
			Image& image = pipeline->images[i];
			image.keyPoints.keyPoints.clear();
#if 1
			ss1 << image.filename << ", ";
#endif
			BufferReader* reader = BufferReaderProvider::getInstance().getBufferReader(image.filename);
			RuntimeTypeBuffer img = reader->read(image.filename);
			delete reader;
			detector->detect(img, image.keyPoints.keyPoints);
#if 1	
			kpt += image.keyPoints.keyPoints.size();
			cnt++;
			if(cnt % 4 == 0) {
				ss2 << kpt << " keypoints"; kpt = 0;
				pipeline->toc(ss1.str(), ss2.str(), N, cnt, id, false); cnt = 0;
				ss1.str("");
				ss2.str("");
				ss1 << "Detecting keypoints with " << detectorType << " on ";
				pipeline->tic(r.begin(), false);
			}
#endif
		}
#if 1
		if(cnt) {
			ss2 << kpt << " keypoints"; kpt = 0;
			pipeline->toc(ss1.str(), ss2.str(), N, cnt, id, false); cnt = 0;
		}
#endif
		delete detector;
	}
	ParallelDetector(FeatureMatchingPipeline* pipeline, DetectorType detectorType) : pipeline(pipeline), detectorType(detectorType) {}
};
#endif

void KeyPointDetectionStage::run(FeatureMatchingPipeline *pipeline) {
	FeatureDetector* detector = FeatureDetectorProvider::getInstance().getDetector(detectorType);
	pipeline->tic();
	std::stringstream ss1, ss2;

	size_t N = pipeline->images.size();
#ifndef WITH_TBB
	for(size_t i = 0; i < N; ++i) {
		Image& image = pipeline->images[i];
		image.keyPoints.keyPoints.clear();
		
		ss1 << "Detecting keypoints with " << detectorType << " on " << image.filename;
		pipeline->tic();
		
		BufferReader* reader = BufferReaderProvider::getInstance().getBufferReader(image.filename);
		RuntimeTypeBuffer img = reader->read(image.filename);
		delete reader;
		detector->detect(img, image.keyPoints.keyPoints);
		ss2 << image.keyPoints.keyPoints.size() << " keypoints";
		pipeline->toc(ss1.str(), ss2.str(), 0, N - i);
		ss1.str(""); ss2.str("");
	}
#else
	tbb::parallel_for(tbb::blocked_range<size_t>(0, N, std::max(N / 16, (size_t)1)), ParallelDetector(pipeline,detectorType));
#endif
	ss1 << "Detecting keypoints with " << detectorType;
	pipeline->toc(ss1.str(), ss2.str());
	delete detector;
}

void KeyPointDetectionStage::saveResults(FeatureMatchingPipeline *pipeline, const std::string &filename) const {
	std::vector<Image>& images = pipeline->images;
	for(size_t i = 0; i < images.size(); ++i) {
		std::string filename = changeExtension(images[i].filename, KEYPOINT_EXTENSION);

		images[i].keyPoints.save(filename);
	}

}

void KeyPointDetectionStage::loadResults(FeatureMatchingPipeline *pipeline, const std::string &filename) {
	std::vector<Image>& images = pipeline->images;
	for(size_t i = 0; i < images.size(); ++i) {
		std::string filename = changeExtension(images[i].filename, KEYPOINT_EXTENSION);
		std::ifstream ifs;

		images[i].keyPoints.load(filename);
	}
}

KeyPointDetectionStage::KeyPointDetectionStage(DetectorType type) : detectorType(type) {
}

void DescriptorExtractionStage::saveResults(FeatureMatchingPipeline *pipeline, const std::string &filename) const {
	auto images = pipeline->images;
	for(size_t i = 0; i < images.size(); ++i) {
		std::string filename = changeExtension(images[i].filename, DESCRIPTOR_EXTENSION);

		images[i].descriptors.save(filename);
		
		// NOTE: in some cases (e.g. using different detector/descriptor setup) we are not able
		// to compute descriptor for each of keypoint, so some of them are removed.
		// So, we need to overwrite keypoint files!
		filename = changeExtension(images[i].filename, KEYPOINT_EXTENSION);

		images[i].keyPoints.save(filename);
	}
	
}

void DescriptorExtractionStage::loadResults(FeatureMatchingPipeline *pipeline, const std::string &filename) {
	std::vector<Image> &images = pipeline->images;
	for(size_t i = 0; i < images.size(); ++i) {
		Image &image = images[i];
		std::string filename = changeExtension(image.filename, DESCRIPTOR_EXTENSION);

		image.descriptors.load(filename);

		assert(image.descriptors.type == descriptorType);

		assert(image.keyPoints.keyPoints.size() == (image.descriptors.mat.getRows()));
	}
}

#ifdef WITH_TBB
class ParallelExtractor {
	FeatureMatchingPipeline* pipeline;
	DescriptorType descriptorType;
public:
	void operator() (const tbb::blocked_range<size_t>& r) const {
	DescriptorExtractor* extractor = DescriptorExtractorProvider::getInstance().getDescriptorExtractor(descriptorType);
		size_t N = pipeline->images.size();
		size_t id = r.begin();
#if 1
		std::stringstream ss1, ss2;
		ss1 << "Extracting " << descriptorType << " descriptors " << " from ";
		size_t cnt = 0;
		pipeline->tic(id, false);
		size_t kpt = 0;
#endif
		for(size_t i = r.begin(); i != r.end(); ++i) {
			Image& image = pipeline->images[i];
#if 1
			ss1 << image.filename << ", ";
#endif
			BufferReader* reader = BufferReaderProvider::getInstance().getBufferReader(image.filename);
			RuntimeTypeBuffer img = reader->read(image.filename);
			delete reader;
		extractor->compute(img, image.keyPoints.keyPoints, image.descriptors.mat);
		image.descriptors.type = descriptorType;

		assert(image.descriptors.mat.getRows() == image.keyPoints.keyPoints.size());
#if 1	
			kpt += image.keyPoints.keyPoints.size();
			cnt++;
			if(cnt % 4 == 0) {
				ss2 << kpt << " descriptors"; kpt = 0;
				pipeline->toc(ss1.str(), ss2.str(), N, cnt, id, false); cnt = 0;
				ss1.str("");
				ss2.str("");
		ss1 << "Extracting " << descriptorType << " descriptors " << " from ";
				pipeline->tic(r.begin(), false);
			}
#endif
		}
#if 1
		if(cnt) {
			ss2 << kpt << " descriptors"; kpt = 0;
			pipeline->toc(ss1.str(), ss2.str(), N, cnt, id, false); cnt = 0;
		}
#endif
		delete extractor;
	}
	ParallelExtractor(FeatureMatchingPipeline* pipeline, DescriptorType descriptorType) : pipeline(pipeline), descriptorType(descriptorType) {}
};
#endif

void DescriptorExtractionStage::run(FeatureMatchingPipeline *pipeline) {
	DescriptorExtractor* extractor = DescriptorExtractorProvider::getInstance().getDescriptorExtractor(descriptorType);
	pipeline->tic();
	std::stringstream ss1, ss2;
	std::vector<Image> &images = pipeline->images;
	
	size_t N = pipeline->images.size();
#ifndef WITH_TBB
	for(size_t i = 0; i < N; ++i) {
		Image& image = pipeline->images[i];
		ss1 << "Extracting " << descriptorType << " descriptors " << " from " << image.filename;
		pipeline->tic();
		
		BufferReader* reader = BufferReaderProvider::getInstance().getBufferReader(image.filename);
		RuntimeTypeBuffer img = reader->read(image.filename);
		delete reader;

		extractor->compute(img, image.keyPoints.keyPoints, image.descriptors.mat);
		image.descriptors.type = descriptorType;

		assert(image.descriptors.mat.getRows() == image.keyPoints.keyPoints.size());
		ss2 << image.descriptors.mat.getRows() << " descriptors";
		pipeline->toc(ss1.str(), ss2.str(), i, N - i);
		ss1.str(""); ss2.str("");
	}
#else
	tbb::parallel_for(tbb::blocked_range<size_t>(0, N, std::max(N / 16,(size_t)1)), ParallelExtractor(pipeline,descriptorType));
#endif

	ss1 << "Extracting " << descriptorType << " descriptors";
	pipeline->toc(ss1.str(), ss2.str());
	delete extractor;
}

DescriptorExtractionStage::DescriptorExtractionStage(DescriptorType type) : descriptorType(type) {
}

void MatchingPlanComputationStage::saveResults(FeatureMatchingPipeline *pipeline, const std::string &filename) const {
	pipeline->matchPlan.save(filename);
}

void MatchingPlanComputationStage::loadResults(FeatureMatchingPipeline *pipeline, const std::string &filename) {
	pipeline->matchPlan.load(filename);
}

void MatchingPlanComputationStage::run(FeatureMatchingPipeline *pipeline) {
	pipeline->tic();
	MatchPlan &matchPlan = pipeline->matchPlan;
	std::vector<Image> &images = pipeline->images;
	size_t N = images.size();

	matchPlan.plan.clear();

	for(size_t i = 0; i < N; ++i) {
		std::deque<uint16_t> query(images[i].keyPoints.keyPoints.size());
		for(size_t j = 0; j < images[i].keyPoints.keyPoints.size(); ++j)	query[j] = j;

		for(size_t j = 0; j < N; ++j) {
			if(i == j)
				continue;
			std::deque<uint16_t> train(images[j].keyPoints.keyPoints.size());
			for(size_t k = 0; k < images[j].keyPoints.keyPoints.size(); ++k)	train[k] = k;

            MatchPlanEntry entry = {i, j, query, train};
            matchPlan.plan.push_back(entry);
		}
	}
	pipeline->toc("Preparing matching plan", "");
}

MatchingStage::MatchingStage(DescriptorType type, size_t responsesPerPoint) : descriptorType(type), responsesPerPoint(responsesPerPoint) {
}
#ifdef WITH_TBB
class ParallelMatcher {
	FeatureMatchingPipeline* pipeline;
	DescriptorType descriptorType;
	size_t responsesPerPoint;
public:
	void operator() (const tbb::blocked_range<size_t>& r) const {
		size_t N = pipeline->images.size();
		size_t S = pipeline->matchPlan.plan.size();
		size_t id = r.begin();
	MatchPlan &matchPlan = pipeline->matchPlan;
	RawMatches &rawMatches = pipeline->rawMatches;
	std::vector<Image> &images = pipeline->images;
#if 1
		std::stringstream ss1, ss2;
		ss1 << "Matched sets ";
		size_t cnt = 0;
		pipeline->tic(id, false);
		size_t kpt = 0;
#endif
		for(size_t i = r.begin(); i != r.end(); ++i) {
			ss1 << i << ", ";
			size_t s = i;
		size_t I = matchPlan.plan[s].queryImg;
		size_t J = matchPlan.plan[s].trainImg;
		auto &query = matchPlan.plan[s];

		assert(I < N && J < N);


		RuntimeTypeBuffer qb(images[I].descriptors.mat);
		RuntimeTypeBuffer tb(images[J].descriptors.mat);

		for(size_t j = 0; j < query.queryFeatures.size(); ++j) {
			memcpy(qb.row<void>(j), images[I].descriptors.mat.row<void>(query.queryFeatures[j]), qb.getRowSize());
		}
		for(size_t j = 0; j < query.trainFeatures.size(); ++j) {
			memcpy(tb.row<void>(j), images[J].descriptors.mat.row<void>(query.trainFeatures[j]), tb.getRowSize());
		}

		DescriptorMatcher* matcher = DescriptorMatcherProvider::getInstance().getMatcher(descriptorType);
		std::vector<std::vector<RawMatch>> ml;
		matcher->knnMatch(qb, tb, ml, responsesPerPoint);


		for(auto& v: ml) {
			std::array<RawMatch, 2> mk;
			for(size_t i = 0; i < v.size(); ++i) {
				mk[i] = v[i];
			}
			rawMatches.matches[s].push_back(mk);
		}

		// It's time to replace indicies
		for(auto& v: rawMatches.matches[s]) {
			for(auto& m: v) {
				m.featureQ = query.queryFeatures[m.featureQ];
				m.featureT = query.trainFeatures[m.featureT];
			}
		}
#if 1	
			cnt++;
			if(cnt % 16 == 0) {
				pipeline->toc(ss1.str(), ss2.str(), S, cnt, id, false); cnt = 0;
				ss1.str("");
				ss1 << "Matched sets ";
				pipeline->tic(r.begin(), false);
			}
#endif
		}
#if 1
		if(cnt) {
			pipeline->toc(ss1.str(), ss2.str(), S, cnt, id, false); cnt = 0;
		}
#endif
	}
	ParallelMatcher(FeatureMatchingPipeline* pipeline, DescriptorType descriptorType, size_t responsesPerPoint) : pipeline(pipeline), descriptorType(descriptorType), responsesPerPoint(responsesPerPoint) {}
};
#endif

void MatchingStage::run(FeatureMatchingPipeline *pipeline) {
	pipeline->tic();
	MatchPlan &matchPlan = pipeline->matchPlan;
	RawMatches &rawMatches = pipeline->rawMatches;
	std::vector<Image> &images = pipeline->images;
	size_t N = images.size();

	assert(matchPlan.plan.size());
	rawMatches.matches.clear();
	rawMatches.matches.resize(matchPlan.plan.size());

	size_t total = 0;
	DescriptorMatcher* matcher = DescriptorMatcherProvider::getInstance().getMatcher(descriptorType);

#ifndef WITH_TBB
	for(size_t s = 0; s < matchPlan.plan.size(); ++s) {
		pipeline->tic();
		size_t I = matchPlan.plan[s].queryImg;
		size_t J = matchPlan.plan[s].trainImg;
		auto &query = matchPlan.plan[s];

		assert(I < N && J < N);

#if 0
		if(!images[I].descriptors.mat.isValid()) {
			std::string filename = changeExtension(images[I].filename, DESCRIPTOR_EXTENSION);

			images[I].descriptors.load(filename);
		}
		if(!images[J].descriptors.mat.isValid()) {
			std::string filename = changeExtension(images[J].filename, DESCRIPTOR_EXTENSION);

			images[J].descriptors.load(filename);
		}
#endif

		RuntimeTypeBuffer qb(images[I].descriptors.mat);
		RuntimeTypeBuffer tb(images[J].descriptors.mat);

		for(size_t j = 0; j < query.queryFeatures.size(); ++j) {
			memcpy(qb.row<void>(j), images[I].descriptors.mat.row<void>(query.queryFeatures[j]), qb.getRowSize());
		}
		for(size_t j = 0; j < query.trainFeatures.size(); ++j) {
			memcpy(tb.row<void>(j), images[J].descriptors.mat.row<void>(query.trainFeatures[j]), tb.getRowSize());
		}

//		matcher->knnMatch(qb, tb, rawMatches.matches[s], responsesPerPoint);
        std::vector<std::vector<RawMatch>> ml;
        matcher->knnMatch(qb, tb, ml, responsesPerPoint);


        for(std::vector<std::vector<RawMatch> >::iterator it = ml.begin(); it != ml.end(); ++it) {
            std::vector<RawMatch>& v = *it;
            std::array<RawMatch, 2> mk;
            for(size_t i = 0; i < v.size() && i < 2 && v[i].isValid(); ++i) {
                mk[i] = v[i];
            }
            rawMatches.matches[s].push_back(mk);
        }
#if 0
		images[I].descriptors.mat = RuntimeTypeBuffer();
		images[J].descriptors.mat = RuntimeTypeBuffer();
#endif

		// It's time to replace indicies
        for(std::deque<std::array<RawMatch, 2>>::iterator it = rawMatches.matches[s].begin(); it != rawMatches.matches[s].end(); ++it) {
            std::array<RawMatch, 2> &v = *it;
            for(std::array<RawMatch, 2>::iterator m = v.begin(); m != v.end() && m->isValid(); ++m) {
				total++;
                m->featureQ = query.queryFeatures[m->featureQ];
                m->featureT = query.trainFeatures[m->featureT];
			}
		}
		std::stringstream ss1, ss2;
		ss1 << "Matching on set " << s << "/" << matchPlan.plan.size();
		pipeline->toc(ss1.str(), ss2.str(), s, matchPlan.plan.size() - s);
	}
#else
	size_t S = matchPlan.plan.size();
	tbb::parallel_for(tbb::blocked_range<size_t>(0, S, std::max(S / 16,(size_t)1)), ParallelMatcher(pipeline,descriptorType, responsesPerPoint));
#endif
	std::stringstream ss;
	ss << total << " matches (non-symmetric)";
	pipeline->toc("Computing raw matches", ss.str());
        delete matcher;
}

void MatchingStage::loadResults(FeatureMatchingPipeline *pipeline, const std::string &filename) {
	pipeline->rawMatches.load(filename);
}

void MatchingStage::saveResults(FeatureMatchingPipeline *pipeline, const std::string &filename) const {
	pipeline->rawMatches.save(filename);
}

#if 0
RandIndexStage::RandIndexStage() {
};

void RandIndexStage::saveResults(FeatureMatchingPipeline *pipeline, const std::string &filename) const {
	std::ofstream os;
	os.open(filename, std::ostream::out);


	size_t N = index.size();

	for(size_t i = 0; i < N; ++i) {
		for(size_t j = 0; j < N; ++j) {
			os << std::setprecision(9) <<  index[i][j] << (j + 1 == N ? "" : ",");
		}
		os << std::endl;
	}
	
}
#endif
#if 0
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

void drawMatches(const std::string &prefix, size_t idxL, size_t idxR, std::vector<std::vector<size_t>> &cls, std::vector<int> &bestA, std::vector<int> &bestB, FeatureMatchingPipeline* pipeline, bool rl, RefinedMatchSet *set = 0) {
	cv::Mat A, B, R;
	auto& images = pipeline->images;
	A = cv::imread(images[idxL].filename);
	B = cv::imread(images[idxR].filename);

	auto szA = A.size();
	auto szB = B.size();

	cv::Size sz(szA.width, szB.height + szA.height);
	R.create(sz, CV_MAKETYPE(A.depth(), 3));

	cv::Mat viewA = R(cv::Rect(0,0,szA.width,szA.height));
	cv::Mat viewB = R(cv::Rect(0, szA.height, szB.width, szB.height));
	A.copyTo(viewA);
	B.copyTo(viewB);

	char filename[1000];
	sprintf(filename, "%s_%s-%s.jpg", prefix.c_str(), images[idxL].filename.c_str(), images[idxR].filename.c_str());

	if(rl){//bestA.size()) {
		for(size_t i = 0; i < images[idxL].keyPoints.keyPoints.size(); ++i) {
	//kp: images[idxL].keyPoints.keyPoints) {
			auto& kp = images[idxL].keyPoints.keyPoints[i];
			size_t idx = cls[idxL][i];
			if(bestA[bestB[idx]] != idx) idx = 0;
			cv::circle(viewA, cv::Point(kp.x, kp.y), 5, cv::Scalar((idx * 67 + 35) % 256, (idx * 7 + 142) % 256, (idx * 217 + 251) % 256), -5);
		}
		for(size_t i = 0; i < images[idxR].keyPoints.keyPoints.size(); ++i) {
	//kp: images[idxL].keyPoints.keyPoints) {
			auto& kp = images[idxR].keyPoints.keyPoints[i];
			size_t idx = cls[idxR][i];
			if(bestB[bestA[idx]] != idx) { idx = 0; } else { idx = bestA[idx]; }
			
			cv::circle(viewB, cv::Point(kp.x, kp.y), 5, cv::Scalar((idx * 67 + 35) % 256, (idx * 7 + 142) % 256, (idx * 217 + 251) % 256), -5);
		}
		if(set) {
			auto& s = *set;
			for(size_t i = 0; i < set->matches.size(); ++i) {
				size_t kpL = idxL < idxR ? set->matches[i].featureA : set->matches[i].featureB;
				size_t kpR = idxL < idxR ? set->matches[i].featureB : set->matches[i].featureA;
				auto& kpA = images[idxL].keyPoints.keyPoints[kpL];
				auto& kpB = images[idxR].keyPoints.keyPoints[kpR];

				size_t idx = cls[idxL][kpL];
				cv::line(R,  cv::Point(kpA.x, kpA.y), cv::Point(kpB.x, kpB.y + szA.height), cv::Scalar((idx * 67 + 35) % 256, (idx * 7 + 142) % 256, (idx * 217 + 251) % 256));
			}
		}
	} else {
	for(size_t i = 0; i < images[idxL].keyPoints.keyPoints.size(); ++i) {
//kp: images[idxL].keyPoints.keyPoints) {
		auto& kp = images[idxL].keyPoints.keyPoints[i];
		size_t idx = cls[idxL][i];
		cv::circle(viewA, cv::Point(kp.x, kp.y), 5, cv::Scalar((idx * 67 + 35) % 256, (idx * 7 + 142) % 256, (idx * 217 + 251) % 256), -5);
	}
	for(size_t i = 0; i < images[idxR].keyPoints.keyPoints.size(); ++i) {
//kp: images[idxL].keyPoints.keyPoints) {
		auto& kp = images[idxR].keyPoints.keyPoints[i];
		size_t idx = cls[idxR][i];
		cv::circle(viewB, cv::Point(kp.x, kp.y), 5, cv::Scalar((idx * 67 + 35) % 256, (idx * 7 + 142) % 256, (idx * 217 + 251) % 256), -5);
	}
	}
	cv::imwrite(filename, R);
	std::cerr << "Writing output to " << filename << std::endl;
}
#endif

#if 0
class ParallelCl {
public:
	void operator() (const tbb::blocked_range<size_t>& r) const {
	RefinedMatches &refinedMatches = pipeline->refinedMatches;
	size_t S = refinedMatches.matchSets.size();
	size_t N = pipeline->images.size();
	std::vector<std::vector<size_t>>&clusters = *cls;
	std::vector<int> clcl;
	size_t id = r.begin();
		std::stringstream ss1, ss2;
		ss1 << "Computed cluster inliers for sets ";
		size_t cnt = 0;
		pipeline->tic(id, false);

	for(size_t s = r.begin(); s < r.end(); ++s) {
		size_t eq = 0, neq = 0, total = 0;
//		ss1 << s << ", ";
		cnt++;
#if 0
		for(size_t i = 0; i < refinedMatches.matchSets[s].matches.size(); ++i) {
			auto& mA = refinedMatches.matchSets[s].matches[i];
			for(size_t j = 0; j < refinedMatches.matchSets[s].matches.size(); ++j) {
				auto& mB = refinedMatches.matchSets[s].matches[j];
				assert(mA.imgA == mB.imgA);
				assert(mA.imgA < N && mA.imgB < N && mB.imgA < N && mB.imgB < N);
				assert(mA.featureA < images[mA.imgA].keyPoints.keyPoints.size());
				assert(mA.featureB < images[mA.imgB].keyPoints.keyPoints.size());
				assert(mB.featureA < images[mB.imgA].keyPoints.keyPoints.size());
				assert(mB.featureB < images[mB.imgB].keyPoints.keyPoints.size());
				bool sameA = clusters[mA.imgA][mA.featureA] == clusters[mB.imgA][mB.featureA];
				bool sameB = clusters[mA.imgB][mA.featureB] == clusters[mB.imgB][mB.featureB];
				if(sameA == sameB) {
					eq++;
				}else {
//					std::cerr << "NEQ" << std::endl;
					neq++;
				}
				total++;
			}
		}
		size_t iA = refinedMatches.matchSets[s].imgA;
		size_t iB = refinedMatches.matchSets[s].imgB;
		std::cerr << "Computed index for " << iA << ":" << iB << std::endl;
		index[iA][iB] = double(eq) / double(total);
		index[iB][iA] = index[iA][iB];
#else
		// A. Get best-best cluster matching.
		auto& v = refinedMatches.matchSets[s].matches;
		size_t kA = clusters[refinedMatches.matchSets[s].imgA].size();
		size_t kB = clusters[refinedMatches.matchSets[s].imgB].size();

		//std::vector<std::vector<int>> clcl(kA);
//		for(size_t i = 0; i < kA; ++i)
//			clcl[i].resize(kB);

		if(clcl.size() < kB * kA) clcl.resize(kB * kA);
		for(size_t i = 0; i < kB * kA; ++i) clcl[i] = 0;

		for(auto m: v) {
			//clcl[clusters[m.imgA][m.featureA]][clusters[m.imgB][m.featureB]]++;
			clcl[clusters[m.imgA][m.featureA]*kB  +clusters[m.imgB][m.featureB]]++;
		}
//		std::vector<int> best_h(K), best_v(K), sum_cl(K), sum_cl2(K);

		std::vector<int> best_A(kB), best_B(kA), sum_A(kA), sum_B(kB);

		for(int i = 0; i < kA; ++i) {
			for(int j = 0; j < kB; ++j) {
			//	if(clcl[i][j] > clcl[i][best_B[i]]) best_B[i] = j;
			//	if(clcl[i][j] > clcl[best_A[j]][j]) best_A[j] = i;
				if(clcl[i*kB+j] > clcl[i*kB+best_B[i]]) best_B[i] = j;
				if(clcl[i*kB+j] > clcl[best_A[j]*kB+j]) best_A[j] = i;

				sum_A[i] += clcl[i*kB+j];
				sum_B[j] += clcl[i*kB+j];
			}
		}
#if 0
		for(int i = 0; i < K; ++i) {
			for(int j = 0; j < K; ++j) {
				if(clcl[i][j] > clcl[i][best_h[i]]) best_h[i] = j;
				if(clcl[i][j] > clcl[best_v[j]][j]) best_v[j] = i;
				sum_cl[i] += clcl[i][j];
				sum_cl2[j] += clcl[i][j];
			}
		}
#endif
#if 0
		for(int i = 0; i < K; ++i) {
			if(best_h[i] >= 0 && best_v[best_h[i]] == i)
				continue;
			double n = clcl[i][best_h[i]];
			double d1 = n / sum_cl[i];
			double d2 = n / sum_cl2[best_h[i]];
			if(d1 > 0.9 && d2 > 0.9) continue;
			best_h[i] = -1;
		}
#endif
			drawMatches("features", v[0].imgA, v[0].imgB, *cls, best_A, best_B, pipeline, true, &refinedMatches.matchSets[s]);
//		std::cerr << "Computed matches for " << v[0].imgA << ":" << v[0].imgB << std::endl;
ss1 << v[0].imgA << ":" << v[0].imgB << ", ";
		int idx = 0;
		for(int i = 0; i < v.size(); ++i) {
			int clA = clusters[v[i].imgA][v[i].featureA];
			int clB = clusters[v[i].imgB][v[i].featureB];
			if(best_B[clA] != clB)
				continue;
			if(best_A[clB] != clA)
				continue;
#if 1
//			if(double(clcl[clA][clB]) / sum_A[clA] < 0.3)
			if(double(clcl[clA*kB+clB]) / sum_A[clA] < 0.2)
				continue;
//			if(double(clcl[clA][clB]) / sum_B[clB] < 0.3)
			if(double(clcl[clA*kB+clB]) / sum_B[clB] < 0.2)
				continue;
#endif
#if 1
//			if(clcl[clA][clB] < 8)
			if(clcl[clA*kB+clB]<20)
				continue;
#endif
			v[idx++] = v[i];
		}
		v.resize(idx);
#endif
			drawMatches("matches", v[0].imgA, v[0].imgB, *cls, best_A, best_B, pipeline, true, &refinedMatches.matchSets[s]);
			if(cnt % 16 == 0) {
				pipeline->toc(ss1.str(), ss2.str(), S, cnt, id, false); cnt = 0;
				ss1.str("");
				ss1 << "Computed cluster inliers for sets  ";
				pipeline->tic(r.begin(), false);
			}
	}
		if(cnt) {
			pipeline->toc(ss1.str(), ss2.str(), S, cnt, id, false); cnt = 0;
		}
	}
	std::vector<std::vector<size_t>> *cls;
	FeatureMatchingPipeline *pipeline;
	ParallelCl(std::vector<std::vector<size_t>> *clusters, FeatureMatchingPipeline *pipeline)
		: cls(clusters), pipeline(pipeline) {};
};


void RandIndexStage::run(FeatureMatchingPipeline *pipeline) {
	pipeline->tic();
	RefinedMatches &refinedMatches = pipeline->refinedMatches;
	size_t N = pipeline->images.size();
	index.resize(N);
	std::deque<Image>& images = pipeline->images;
	for(size_t i = 0; i < N; ++i) index[i].resize(N);

	std::vector<std::vector<size_t>> clusters(N);
	for(size_t i = 0; i < N; ++i) {
		size_t M = images[i].keyPoints.keyPoints.size();
		auto& kp = images[i].keyPoints.keyPoints;
		std::cerr << "Computing clusters for " << images[i].filename << std::endl;
		size_t K = std::max((int)M / 500, 5);

		std::vector<double> cx(K), cy(K);
		std::vector<size_t> cl(M);
		std::vector<size_t> cnt(K);

		for(size_t j = 0; j < M; ++j)
			cl[j] = rand() % K;

		size_t I = 1000000;
		bool stop = false;
		for(size_t j = 0; j < I && !stop; ++j) {
			stop = true;
			cx.clear(); cx.resize(K);
			cy.clear(); cy.resize(K);
			cnt.clear();cnt.resize(K);

			for(size_t k = 0; k < M; ++k) {
				cnt[cl[k]]++;
				cx[cl[k]] += kp[k].x;
				cy[cl[k]] += kp[k].y;
			}

			for(size_t k = 0; k < K; ++k) {
				cx[k] /= cnt[k];
				cy[k] /= cnt[k];
			}

			for(size_t k = 0; k < K; ++k) {
				if(cnt[k] == 0) {
					size_t idx;
					cx[k] = kp[idx = (rand() % M)].x;
					cy[k] = kp[idx].y;
			//		std::cerr << "Re-init cluster " << k << std::endl;
				}
			}

			for(size_t k = 0; k < M; ++k) {
				double mindist = 1e100;
				size_t minidx = 0;
				for(size_t l = 0; l < K; ++l) {
					double dx = (cx[l] - kp[k].x);
					double dy = (cy[l] - kp[k].y);
					double len = dx * dx + dy * dy;
					if(mindist > len) {
						mindist = len;
						minidx = l;
					}
				}
				if(cl[k] != minidx) {
					cl[k] = minidx;
					stop = false;
				}
			}
		}
		clusters[i] =(cl);
		std::ofstream os;
		char fname[10000];
		sprintf(fname, "%s.csv", images[i].filename.c_str());
		os.open(fname, std::ofstream::out);
		for(size_t k = 0; k < M; ++k)
			os << clusters[i][k] << ", ";
		assert(clusters[i].size() == images[i].keyPoints.keyPoints.size());
	}

	pipeline->toc("Computing clusters", "");

	pipeline->tic();
	size_t S = refinedMatches.matchSets.size();
	tbb::parallel_for(tbb::blocked_range<size_t>(0, S, std::max(S / 16,(size_t)1)), ParallelCl(&clusters,pipeline));
	pipeline->toc("Computing cluster inliers", "");

}
#endif
RefineMatchesStage::RefineMatchesStage(bool symmetric, double scaleThreshold) :
	symmetric(symmetric), scaleThreshold(scaleThreshold) {
}

void RefineMatchesStage::run(FeatureMatchingPipeline *pipeline) {
	pipeline->tic();
	std::vector<Image> &images = pipeline->images;
	MatchPlan &matchPlan = pipeline->matchPlan;
	RawMatches &rawMatches = pipeline->rawMatches;
	RefinedMatches &refinedMatches = pipeline->refinedMatches;

	// [i][j][k][l]
	// i - query index
	// j - train index
	// k - feature index
	// l - match index (<= 2)
	// Yep, it is too ugly
	std::cerr << "Prealloc accumulator" << std::endl;
	std::deque<std::deque<std::deque<std::array<RawMatch,2> > > > accumulator;

	size_t N = pipeline->images.size();

	accumulator.resize(N);
	for(size_t i = 0; i < N; ++i) {
		accumulator[i].resize(N);
		for(size_t j = 0; j < N; ++j) {
			accumulator[i][j].resize(images[i].keyPoints.keyPoints.size());
#if 0
			for(size_t k = 0; k < accumulator[i][j].size(); ++k) {
//				accumulator[i][j][k].resize(2);
//				accumulator[i][j][k][1].distance = 1000.0;
//				accumulator[i][j][k][0].distance = 1000.0;
			}
#endif
		}
	}
	std::cerr << "Accumulator alloc'ed" << std::endl;
	std::cerr << "Populate accumulator" << std::endl;


//	std::ifstream ifs;
//	ifs.open("matches_raw.txt", std::ifstream::in);
	size_t S = matchPlan.plan.size();
//	ifs >> S;
//	assert(S == matchPlan.plan.size());

	for(size_t s = 0; s < S; ++s) {
		size_t query = matchPlan.plan[s].queryImg;
		size_t train = matchPlan.plan[s].trainImg;

		size_t L = rawMatches.matches[s].size() , K;
//		ifs >> L;		

		for(size_t i = 0; i < L; ++i) {
//			ifs >> K;
			K = rawMatches.matches[s][i].size();
			for(size_t j = 0; j < K && rawMatches.matches[s][i][j].isValid(); ++j) {
//				RawMatch dm;
//				ifs >> dm;
				RawMatch dm = rawMatches.matches[s][i][j];
				if(dm.distance < accumulator[query][train][dm.featureQ][0].distance) {
					accumulator[query][train][dm.featureQ][1] = accumulator[query][train][dm.featureQ][0];
					accumulator[query][train][dm.featureQ][0] = dm;
				} else if( dm.distance < accumulator[query][train][dm.featureQ][1].distance) {
					accumulator[query][train][dm.featureQ][1] = dm;
				}
			}
		}
//		rawMatches.matches[s].clear();
//		rawMatches.matches[s].shrink_to_fit();
	}
//	rawMatches.matches.clear();
//	rawMatches.matches.shrink_to_fit();
	std::cerr << "Populated accumulator" << std::endl;
	std::cerr << "Ratio filtering" << std::endl;

	std::deque<std::deque<std::deque<RawMatch> > > ratioInliers(N);
	for(size_t i = 0; i < N; ++i) {
		ratioInliers[i].resize(N);
		for(size_t j = 0; j < N; ++j)
			ratioInliers[i][j].resize(images[i].keyPoints.keyPoints.size());
	}

	for(size_t q = 0; q < N; ++q) {
		for(size_t t = 0; t < N; ++t) {
			if(t == q) continue;

			for(size_t k = 0; k < images[q].keyPoints.keyPoints.size(); ++k) {
				if(accumulator[q][t][k][0].distance / accumulator[q][t][k][1].distance < 0.9)
					ratioInliers[q][t][k] = (accumulator[q][t][k][0]);
			}
			accumulator[q][t].clear();
			accumulator[q][t].shrink_to_fit();
		}
	}
	std::cerr << "Ratio filtered" << std::endl;

	accumulator.clear(); accumulator.shrink_to_fit();
	std::cerr << "Symmetry filtering" << std::endl;

	// The second step is to remove non-symmetric matches
	for(size_t q = 0; q < N; ++q) {
		for(size_t t = 0; t < N; ++t) {
			for(size_t k = 0; k < images[q].keyPoints.keyPoints.size(); ++k) {
				RawMatch& dm = ratioInliers[q][t][k];
				if(dm.featureT != RawMatch::INVALID_MARKER && dm.featureQ != RawMatch::INVALID_MARKER && ratioInliers[t][q][dm.featureT].featureT != static_cast<int>(k))
					dm.featureT = dm.featureQ = RawMatch::INVALID_MARKER;
			}
		}
	}
	std::cerr << "Symmetry filtered" << std::endl;

	// Now we can compose final match set
	size_t total_matches = 0;
	refinedMatches.matchSets.clear();
	for(size_t i = 0; i < N; ++i) {
		for(size_t j = i + 1; j < N; ++j) {
			std::deque<Match> matches;
			for(size_t k = 0; k < images[i].keyPoints.keyPoints.size(); ++k) {
				RawMatch& dm = ratioInliers[i][j][k];
				if(ratioInliers[i][j][k].featureT != RawMatch::INVALID_MARKER && ratioInliers[i][j][k].featureQ != RawMatch::INVALID_MARKER)
					matches.push_back(Match(i, j, dm.featureQ, dm.featureT, dm.distance));
			}
			if(matches.size()) {
				refinedMatches.matchSets.push_back(RefinedMatchSet(i, j, matches));
				total_matches += matches.size();
			}
		}
	}
	std::stringstream ss;
	ss << total_matches << " matches (symmetric)";
	pipeline->toc("Filtering matches", ss.str());
}

void RefineMatchesStage::saveResults(FeatureMatchingPipeline *pipeline, const std::string &filename) const {
	pipeline->refinedMatches.save(filename);
}

void RefineMatchesStage::loadResults(FeatureMatchingPipeline *pipeline, const std::string &filename) {
	pipeline->refinedMatches.load(filename);
}

VsfmWriterStage::VsfmWriterStage(bool sortFeatures) : sortFeatures(sortFeatures) {
}

std::string file_name(const std::string &full_name) {
	std::string name = full_name;
	size_t from = 0;
	for(size_t i = 0; i < full_name.size(); ++i)
		if(full_name[i] == '/' || full_name[i] == '\\')
			from = i + 1;
	std::string res;
	res.resize(full_name.size() - from);
	for(size_t i = from, j = 0; i < full_name.size(); ++i, ++j)
		res[j] = full_name[i];
	return res;
}

void VsfmWriterStage::saveResults(FeatureMatchingPipeline *pipeline, const std::string &filename) const {
	size_t N = pipeline->images.size();
	std::vector<std::vector<SiftFeature> > features(N);

	// 1. assign importance to features 
	// 2. sort features by their importance (?)
	// 3. move backwards features without matches (?)
	// 4. write .SIFT
	std::vector<Image> &images = pipeline->images;
	RefinedMatches &refinedMatches = pipeline->refinedMatches;

	uint8_t data[128] = {0}; // Just do not care about data

	for(size_t i = 0; i < N; ++i) {
		features[i].resize(images[i].keyPoints.keyPoints.size());
		for(size_t k = 0; k < images[i].keyPoints.keyPoints.size(); ++k) {
			features[i][k] = SiftFeature(
					images[i].keyPoints.keyPoints[k].x,
					images[i].keyPoints.keyPoints[k].y,
					data,
					images[i].keyPoints.keyPoints[k].size,
					images[i].keyPoints.keyPoints[k].angle,
					0.0,
					0.0);

		}
	}

	for(size_t i = 0; i < refinedMatches.matchSets.size(); ++i) {
		for(size_t j = 0; j < refinedMatches.matchSets[i].matches.size(); ++j) {
			Match& m = refinedMatches.matchSets[i].matches[j];
			features[m.imgA][m.featureA].importance += 1.0;
			features[m.imgB][m.featureB].importance += 1.0;
		}
	}

	std::vector<std::vector<size_t> > reordering(N);
	std::vector<std::vector<size_t> > reordering_rev(N);
	for(size_t i = 0; i < N; ++i) {
		size_t M;
		reordering[i].resize(M = images[i].keyPoints.keyPoints.size());
		reordering_rev[i].resize(M);
		for(size_t j = 0; j < M; ++j)
			reordering[i][j] = j;
		if(sortFeatures) {
			std::sort(reordering[i].begin(), reordering[i].end(),
					[&](const size_t &a, const size_t &b) {
					return features[i][a].importance > features[i][b].importance; });
		}
		for(size_t j = 0; j < M; ++j)
			reordering_rev[i][reordering[i][j]] = j;
	}

	for(size_t i = 0; i < N; ++i) {
		std::vector<SiftFeature> write_features(features[i].size());
		for(size_t j = 0; j < features[i].size(); ++j) {
			write_features[j] = features[i][reordering[i][j]];
		}

		std::string file = changeExtension(images[i].filename, SIFT_EXTENSION);

		std::ofstream of;
		of.open(file.c_str(), std::ofstream::out);
		VsfmSiftIO::writeAscii(of, write_features);
	}

	std::ofstream ofs;
	ofs.open(filename.c_str(), std::ofstream::out);
	assert(ofs);

	for(size_t i = 0; i < refinedMatches.matchSets.size(); ++i) {
		RefinedMatchSet& set = refinedMatches.matchSets[i];
		ofs << file_name(images[set.imgA].filename) << std::endl << file_name(images[set.imgB].filename) << std::endl;

		ofs << set.matches.size() << std::endl;

		for(std::deque<Match>::iterator m = set.matches.begin(); m != set.matches.end(); ++m) {
			assert(m->imgA == set.imgA);
			ofs << reordering_rev[m->imgA][m->featureA] << " ";
		}
		ofs << std::endl;

		for(std::deque<Match>::iterator m = set.matches.begin(); m != set.matches.end(); ++m) {
			assert(m->imgB == set.imgB);
			ofs << reordering_rev[m->imgB][m->featureB] << " ";
		}
		ofs << std::endl;
	}
}

void FeatureMatchingPipeline::tic(size_t thread_id, bool level) {
#ifdef WITH_TBB
	tbb::spin_mutex::scoped_lock lock(mutex);
#endif
	if(level) {
		if(tics.size() == 0) {
			for(size_t i = 0; i < 108; ++i)
				std::cerr << "-";
			std::cerr << std::endl;
		}
		tic_data data;
		data.thread_tics[thread_id] = clock();//std::chrono::high_resolution_clock::now();
		tics.push(data);
	} else {
		tics.top().thread_tics[thread_id] = clock(); //std::chrono::high_resolution_clock::now();
	}
#if 0
	totals.resize(tics.size() + 1);
	counts.resize(tics.size() + 1);
	totals[tics.size()] = 0;
	counts[tics.size()] = 0;
#endif

}

void FeatureMatchingPipeline::toc(const std::string &name, const std::string &evt, size_t thread_id, bool level) {
#ifdef WITH_TBB
	tbb::spin_mutex::scoped_lock lock(mutex);
#endif
#if 0
	auto toc = std::chrono::high_resolution_clock::now();
#else
	size_t toc = clock();
#endif
	if(level) {
		tic_data tic = tics.top();
		tics.pop();

#if 0
		auto ns = std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic.thread_tics[thread_id]).count();
#else
		size_t ns = (toc - tic.thread_tics[thread_id]) / (CLOCKS_PER_SEC / 1000);
#endif
		std::cerr << std::setw(64) << name << std::setw(32) << evt << std::setw(10) << ns << "ms" << std::endl;
	
		if(tics.size() == 0) {
			for(size_t i = 0; i < 108;++i)
				std::cerr << "-";
			std::cerr << std::endl;
		}
	} else {
#if 0
		auto& tic = tics.top();
		auto tict = tic.thread_tics[thread_id];
		auto ns = std::chrono::duration_cast<std::chrono::milliseconds>(toc - tict).count();
#else
		tic_data& tic = tics.top();
		size_t tict = tic.thread_tics[thread_id];
		size_t ns = toc - tict;
#endif
		std::cerr << std::setw(64) << name << std::setw(32) << evt << std::setw(10) << ns << "ms" << std::endl;
	}
}

void FeatureMatchingPipeline::toc(const std::string &name, const std::string &evt, size_t curr, size_t rem, size_t thread_id, bool level) {
#ifdef WITH_TBB
	tbb::spin_mutex::scoped_lock lock(mutex);
#endif

	if(level) {
#if 0
		auto toc = std::chrono::high_resolution_clock::now();
		auto tic = tics.top();
		tics.pop();
	
		auto ns = std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic.thread_tics[thread_id]).count();
#else
		size_t toc = clock();
		tic_data& tic = tics.top();

		size_t ns = (toc - tic.thread_tics[thread_id]) / (CLOCKS_PER_SEC / 1000);
		tics.pop();
#endif


#if 0
		size_t t = totals[tics.size() ] += ns;
		size_t c = ++counts[tics.size() ];
		double one = double(t)/double(c);
		int rs = int(rem * one / 1e3);
		int rm = rs / 60; rs = rs % 60;
		int rh = rm / 60; rm = rm % 60;


		std::cerr << std::setw(64) << name << std::setw(32) << evt << std::setw(10) << ns << "ms (" << rh << "h " << rm << "m " << rs << "s left)" << " [ " << one << " ] " <<  std::endl;
	
		if(tics.size() == 0) {
			for(size_t i = 0; i < 108;++i)
				std::cerr << "-";
			std::cerr << std::endl;
		}
#else
		std::cerr << std::setw(64) << name << std::setw(32) << evt << std::setw(10) << ns << "ms"  <<  std::endl;
#endif
	} else {
#if 0
		auto toc = std::chrono::high_resolution_clock::now();
		auto& tic = tics.top();

		auto ns = std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic.thread_tics[thread_id]).count();
#else
		size_t toc = clock();
		tic_data& tic = tics.top();
		size_t ns = (toc - tic.thread_tics[thread_id]) / (CLOCKS_PER_SEC / 1000);
#endif
		size_t total = 0, count = 0;
		tic.thread_totals[thread_id] += ns;
		tic.thread_counts[thread_id]+= rem;
#if 0
		for(auto v: tic.thread_totals) total += v.second;
#endif
#if 0
		total = std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic.thread_tics[~(size_t)0]).count();
#else
		total = (toc - tic.thread_tics[~(size_t)0]);
#endif
		for(std::map<size_t,size_t>::iterator v = tic.thread_counts.begin(); v != tic.thread_counts.end(); ++v) count += v->second;

		double one = double(total)/double(count);
		int rs = int((curr - count) * one / 1e3);
		int rm = rs / 60; rs = rs % 60;
		int rh = rm / 60; rm = rm % 60;


		std::cerr << std::setw(64) << name << std::setw(32) << evt << std::setw(10) << ns << "ms (" << rh << "h " << rm << "m " << rs << "s left)" << " [ " << one << " ] " <<  std::endl;

	}
}

