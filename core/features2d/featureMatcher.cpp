#include "featureMatcher.h"

#include <cassert>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "featureDetectorProvider.h"
#include "descriptorExtractorProvider.h"
#include "descriptorMatcherProvider.h"
#include "bufferReaderProvider.h"
#include "vsfmIo.h"

//#include "openCvKeyPointsWrapper.h"


const char* KEYPOINT_EXTENSION = "keypoints";
const char* DESCRIPTOR_EXTENSION = "descriptors.xml";
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
	for(auto ps: pipeline)
		ps->~FeatureMatchingPipelineStage();
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



void KeyPointDetectionStage::run(FeatureMatchingPipeline *pipeline) {
	FeatureDetector* detector = FeatureDetectorProvider::getInstance().getDetector(detectorType, detectorsParams);
	size_t N = pipeline->images.size();

	for(size_t i = 0; i < N; ++i) {
		Image& image = pipeline->images[i];
		std::cerr << "Computing keypoints for " << image.filename << std::endl;
		image.keyPoints.keyPoints.clear();
#if 0
		image.img = cv::imread(image.filename, CV_LOAD_IMAGE_GRAYSCALE);

		std::vector<cv::KeyPoint> kps;
		detector->detect(image.img, kps);
		image.keyPoints.keyPoints.clear();
		for(auto kp: kps)
			image.keyPoints.keyPoints.push_back(kp);
#else
		BufferReader* reader = BufferReaderProvider::getInstance().getBufferReader(image.filename);
		RuntimeTypeBuffer img = reader->read(image.filename);
		delete reader;
#if 0
		RuntimeTypeBuffer img(convert(cv::imread(image.filename, CV_LOAD_IMAGE_GRAYSCALE)));
#endif
		detector->detect(img, image.keyPoints.keyPoints);
#endif

		std::cerr << "Detected " << image.keyPoints.keyPoints.size() << " features on " << image.filename << std::endl;
	}
	delete detector;
}

void KeyPointDetectionStage::saveResults(FeatureMatchingPipeline *pipeline, const std::string &filename) const {
	std::vector<Image>& images = pipeline->images;
	for(size_t i = 0; i < images.size(); ++i) {
		std::string filename = changeExtension(images[i].filename, KEYPOINT_EXTENSION);
		std::cerr << "Saving keypoints to " << filename << std::endl;

		images[i].keyPoints.save(filename);
	}

}

void KeyPointDetectionStage::loadResults(FeatureMatchingPipeline *pipeline, const std::string &filename) {
	std::vector<Image>& images = pipeline->images;
	for(size_t i = 0; i < images.size(); ++i) {
		std::string filename = changeExtension(images[i].filename, KEYPOINT_EXTENSION);
		std::ifstream ifs;
		std::cerr << "Loading keypoints from " << filename << std::endl;

		images[i].keyPoints.load(filename);

		std::cerr << "Loaded " << images[i].keyPoints.keyPoints.size() << " keypoints from " << filename << std::endl;
	}
}

KeyPointDetectionStage::KeyPointDetectionStage(DetectorType type, DetectorsParams params) : detectorType(type), detectorsParams(params) {
}

void DescriptorExtractionStage::saveResults(FeatureMatchingPipeline *pipeline, const std::string &filename) const {
	std::vector<Image> &images = pipeline->images;
	for(size_t i = 0; i < images.size(); ++i) {
		std::string filename = changeExtension(images[i].filename, DESCRIPTOR_EXTENSION);
		std::cerr << "Saving descriptors to " << filename << std::endl;

		images[i].descriptors.save(filename);
		
		// NOTE: in some cases (e.g. using different detector/descriptor setup) we are not able
		// to compute descriptor for each of keypoint, so some of them are removed.
		// So, we need to overwrite keypoint files!
		filename = changeExtension(images[i].filename, KEYPOINT_EXTENSION);
		std::cerr << "Saving keypoints to " << filename << std::endl;

		images[i].keyPoints.save(filename);
	}
	
}

void DescriptorExtractionStage::loadResults(FeatureMatchingPipeline *pipeline, const std::string &filename) {
	std::vector<Image> &images = pipeline->images;
	for(size_t i = 0; i < images.size(); ++i) {
		Image &image = images[i];
		std::string filename = changeExtension(image.filename, DESCRIPTOR_EXTENSION);
		std::cerr << "Loading descriptors from " << filename << std::endl;

		image.descriptors.load(filename);

		assert(image.descriptors.type == descriptorType);
		std::cerr << "Loaded " << image.descriptors.mat.getRows() << " descriptors from " << filename << std::endl;

		assert(image.keyPoints.keyPoints.size() == (image.descriptors.mat.getRows()));
	}
}


void DescriptorExtractionStage::run(FeatureMatchingPipeline *pipeline) {
	DescriptorExtractor* extractor = DescriptorExtractorProvider::getInstance().getDescriptorExtractor(descriptorType, detectorsParams);
	size_t N = pipeline->images.size();
	for(size_t i = 0; i < N; ++i) {
		Image& image = pipeline->images[i];
		std::cerr << "Computing descriptors for " << image.filename << std::endl;
#if 0
		cv::Mat img = cv::imread(image.filename, CV_LOAD_IMAGE_GRAYSCALE);
		image.descriptors.type = descriptorType;

		cv::Mat mat;
		std::vector<cv::KeyPoint> kps;
		for(auto kp: image.keyPoints.keyPoints)
			kps.push_back((cv::KeyPoint)kp);

		assert(kps.size() == image.keyPoints.keyPoints.size());

		extractor->compute(img, kps, mat);//image.keyPoints.keyPoints, mat);
		image.keyPoints.keyPoints.clear();
		for(auto kp: kps)
			image.keyPoints.keyPoints.push_back(kp);

		image.descriptors.mat = RuntimeTypeBuffer(mat);
#else
		BufferReader* reader = BufferReaderProvider::getInstance().getBufferReader(image.filename);
		RuntimeTypeBuffer img = reader->read(image.filename);
		delete reader;
#if 0
		RuntimeTypeBuffer img(convert(cv::imread(image.filename, CV_LOAD_IMAGE_GRAYSCALE)));
#endif
		extractor->compute(img, image.keyPoints.keyPoints, image.descriptors.mat);
#endif

		assert(image.descriptors.mat.getRows() == image.keyPoints.keyPoints.size());
		std::cerr << "Computed " << image.descriptors.mat.getRows() << " descriptors on " << image.filename << std::endl;
	}
	delete extractor;
}

DescriptorExtractionStage::DescriptorExtractionStage(DescriptorType type, DetectorsParams params) : descriptorType(type), detectorsParams(params) {
}

void MatchingPlanComputationStage::saveResults(FeatureMatchingPipeline *pipeline, const std::string &filename) const {
	std::cerr << "Saving match plan from " << filename << std::endl;	
	pipeline->matchPlan.save(filename);
}

void MatchingPlanComputationStage::loadResults(FeatureMatchingPipeline *pipeline, const std::string &filename) {
	std::cerr << "Loading match plan from " << filename << std::endl;	
	pipeline->matchPlan.load(filename);
}

void MatchingPlanComputationStage::run(FeatureMatchingPipeline *pipeline) {
	std::cerr << "Creating match plan" << std::endl;
	MatchPlan &matchPlan = pipeline->matchPlan;
	std::vector<Image> &images = pipeline->images;
	size_t N = images.size();

	matchPlan.plan.clear();

	for(size_t i = 0; i < N; ++i) {
		std::vector<size_t> query(images[i].keyPoints.keyPoints.size());
		for(size_t j = 0; j < images[i].keyPoints.keyPoints.size(); ++j)	query[j] = j;

		for(size_t j = 0; j < N; ++j) {
			if(i == j)
				continue;
			std::vector<size_t> train(images[j].keyPoints.keyPoints.size());
			for(size_t k = 0; k < images[j].keyPoints.keyPoints.size(); ++k)	train[k] = k;
			matchPlan.plan.push_back({i, j, query, train});
		}
	}
	std::cerr << "Created match plan with " << matchPlan.plan.size() << " entries" << std::endl;
}


MatchingStage::MatchingStage(DescriptorType type, size_t responsesPerPoint) : descriptorType(type), responsesPerPoint(responsesPerPoint) {
}

void MatchingStage::run(FeatureMatchingPipeline *pipeline) {
	std::cerr << "Computing raw matches" << std::endl;

	MatchPlan &matchPlan = pipeline->matchPlan;
	RawMatches &rawMatches = pipeline->rawMatches;
	std::vector<Image> &images = pipeline->images;
	size_t N = images.size();

	assert(matchPlan.plan.size());
	rawMatches.matches.clear();
	rawMatches.matches.resize(matchPlan.plan.size());

	for(size_t s = 0; s < matchPlan.plan.size(); ++s) {
		size_t I = matchPlan.plan[s].queryImg;
		size_t J = matchPlan.plan[s].trainImg;
		auto &query = matchPlan.plan[s];

		assert(I < N && J < N);

#if 0
		// Now we need to subset descriptors
		cv::Mat qd = (cv::Mat)images[I].descriptors.mat;
		cv::Mat td = (cv::Mat)images[J].descriptors.mat;
		cv::Mat qd2 = (cv::Mat)images[I].descriptors.mat;
		cv::Mat td2 = (cv::Mat)images[J].descriptors.mat;
		
		qd.rows = query.queryFeatures.size();
		td.rows = query.trainFeatures.size();

		for(size_t j = 0; j < query.queryFeatures.size(); ++j) {
			qd2.row(query.queryFeatures[j]).copyTo(qd.row(j));
		}
		for(size_t j = 0; j < query.trainFeatures.size(); ++j) {
			td2.row(query.trainFeatures[j]).copyTo(td.row(j));
		}

		// Some descriptors (ORB, BRISK, SIFTGPU) require specific matchers, 
		// otherwise we use FLANN-based matcher
//		std::vector<cv::DMatch> matches;
//		
		cv::DescriptorMatcher* matcher = DescriptorMatcherProvider::getDescriptorMatcher(descriptorType);
		std::vector<std::vector<cv::DMatch>> matches;
		matcher->knnMatch(qd, td, matches, responsesPerPoint);

		rawMatches.matches[s].clear();
		rawMatches.matches[s].resize(matches.size());
		for(size_t idx = 0; idx < matches.size(); ++idx) {
			for(auto m: matches[idx]) {
				rawMatches.matches[s][idx].push_back(RawMatch(m, I, J));
			}
		}
		delete matcher;
#else
		RuntimeTypeBuffer qb(images[I].descriptors.mat);
		RuntimeTypeBuffer tb(images[J].descriptors.mat);

		for(size_t j = 0; j < query.queryFeatures.size(); ++j) {
			memcpy(qb.row<void>(j), images[I].descriptors.mat.row<void>(query.queryFeatures[j]), qb.getRowSize());
		}
		for(size_t j = 0; j < query.trainFeatures.size(); ++j) {
			memcpy(tb.row<void>(j), images[J].descriptors.mat.row<void>(query.trainFeatures[j]), qb.getRowSize());
		}

		DescriptorMatcher* matcher = DescriptorMatcherProvider::getInstance().getMatcher(descriptorType);
		matcher->knnMatch(qb, tb, rawMatches.matches[s], responsesPerPoint);

		for(auto v: rawMatches.matches[s]) {
			for(auto rm: v) {
				rm.imgQ = I;
				rm.imgT = J;
			}
		}
#endif

		// It's time to replace indicies
		for(auto v: rawMatches.matches[s]) {
			for(auto m: v) {
				m.featureQ = query.queryFeatures[m.featureQ];
				m.featureT = query.trainFeatures[m.featureT];
			}
		}

	}
}

void MatchingStage::loadResults(FeatureMatchingPipeline *pipeline, const std::string &filename) {
	std::cerr << "Loading raw matches from " << filename << std::endl;
	pipeline->rawMatches.load(filename);
}

void MatchingStage::saveResults(FeatureMatchingPipeline *pipeline, const std::string &filename) const {
	std::cerr << "Saving raw matches to " << filename << std::endl;
	pipeline->rawMatches.save(filename);
}

RefineMatchesStage::RefineMatchesStage(bool symmetric, double scaleThreshold) :
	symmetric(symmetric), scaleThreshold(scaleThreshold) {
}

void RefineMatchesStage::run(FeatureMatchingPipeline *pipeline) {
	std::cerr << "Refining matches" << std::endl;

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
#if 0
	std::vector<std::vector<std::vector<std::vector<cv::DMatch>>>> accumulator;
#else
	std::vector<std::vector<std::vector<std::vector<RawMatch>>>> accumulator;
#endif

	size_t N = pipeline->images.size();

	accumulator.resize(N);
	for(size_t i = 0; i < N; ++i) {
		accumulator[i].resize(N);
		for(size_t j = 0; j < N; ++j) {
			accumulator[i][j].resize(images[i].keyPoints.keyPoints.size());
			for(size_t k = 0; k < accumulator[i][j].size(); ++k) {
				accumulator[i][j][k].resize(2);
				accumulator[i][j][k][1].distance = 1000.0;
				accumulator[i][j][k][0].distance = 1000.0;
			}
		}
	}

	for(size_t s = 0; s < matchPlan.plan.size(); ++s) {
		size_t query = matchPlan.plan[s].queryImg;
		size_t train = matchPlan.plan[s].trainImg;

		for(size_t i = 0; i < rawMatches.matches[s].size(); ++i) {
			for(size_t j = 0; j < rawMatches.matches[s][i].size(); ++j) {
#if 0
				cv::DMatch dm = rawMatches.matches[s][i][j];
				if(dm.distance < accumulator[query][train][dm.queryIdx][0].distance) {
					accumulator[query][train][dm.queryIdx][1] = accumulator[query][train][dm.queryIdx][0];
					accumulator[query][train][dm.queryIdx][0] = dm;
				} else if( dm.distance < accumulator[query][train][dm.queryIdx][1].distance) {
					accumulator[query][train][dm.queryIdx][1] = dm;
				}
#else
				RawMatch dm = rawMatches.matches[s][i][j];
				if(dm.distance < accumulator[query][train][dm.featureQ][0].distance) {
					accumulator[query][train][dm.featureQ][1] = accumulator[query][train][dm.featureQ][0];
					accumulator[query][train][dm.featureQ][0] = dm;
				} else if( dm.distance < accumulator[query][train][dm.featureQ][1].distance) {
					accumulator[query][train][dm.featureQ][1] = dm;
				}
#endif
			}
		}
	}
	std::cerr << "Reshape finished" << std::endl;

#if 0
	std::vector<std::vector<std::vector<cv::DMatch>>> ratioInliers(N);
#else
	std::vector<std::vector<std::vector<RawMatch>>> ratioInliers(N);
#endif
	for(size_t i = 0; i < N; ++i) {
		ratioInliers[i].resize(N);
		for(size_t j = 0; j < N; ++j)
			ratioInliers[i][j].resize(images[i].keyPoints.keyPoints.size());
	}

	for(size_t q = 0; q < N; ++q) {
		for(size_t t = 0; t < N; ++t) {
			if(t == q) continue;

			for(size_t k = 0; k < images[q].keyPoints.keyPoints.size(); ++k) {
				if(accumulator[q][t][k][0].distance / accumulator[q][t][k][1].distance < 1.9)
					ratioInliers[q][t][k] = (accumulator[q][t][k][0]);
			}

		}
	}

	accumulator.clear(); accumulator.shrink_to_fit();

	// The second step is to remove non-symmetric matches
	for(size_t q = 0; q < N; ++q) {
		for(size_t t = 0; t < N; ++t) {
			for(size_t k = 0; k < images[q].keyPoints.keyPoints.size(); ++k) {
#if 0
				cv::DMatch& dm = ratioInliers[q][t][k];
#else
				RawMatch& dm = ratioInliers[q][t][k];
#endif
#if 0
				if(dm.trainIdx >= 0 && dm.queryIdx >= 0 && ratioInliers[t][q][dm.trainIdx].trainIdx != static_cast<int>(k))
					dm.trainIdx = dm.queryIdx = -1;
#else
				if(dm.featureT != ~(size_t)0 && dm.featureQ != ~(size_t)0 && ratioInliers[t][q][dm.featureT].featureT != static_cast<int>(k))
					dm.featureT = dm.featureQ = ~(size_t)0;
#endif
			}
		}
	}

	// Now we can compose final match set
	size_t total_matches = 0;
	refinedMatches.matchSets.clear();
	for(size_t i = 0; i < N; ++i) {
		for(size_t j = i + 1; j < N; ++j) {
			std::vector<Match> matches;
			for(size_t k = 0; k < images[i].keyPoints.keyPoints.size(); ++k) {
#if 0
				cv::DMatch& dm = ratioInliers[i][j][k];
				if(ratioInliers[i][j][k].trainIdx >= 0 && ratioInliers[i][j][k].queryIdx >= 0)
					matches.push_back(Match(i, j, dm.queryIdx, dm.trainIdx, dm.distance));
#else
				RawMatch& dm = ratioInliers[i][j][k];
				if(ratioInliers[i][j][k].featureT != ~(size_t)0 && ratioInliers[i][j][k].featureQ != ~(size_t)0)
					matches.push_back(Match(i, j, dm.featureQ, dm.featureT, dm.distance));
#endif
			}
			if(matches.size()) {
				refinedMatches.matchSets.push_back(RefinedMatchSet(i, j, matches));
				total_matches += matches.size();
			}
		}
	}
	std::cerr << "Refined matches " << total_matches << " remaining" << std::endl;
}

void RefineMatchesStage::saveResults(FeatureMatchingPipeline *pipeline, const std::string &filename) const {
	std::cerr << "Saving refined matches to " << filename << std::endl;
	pipeline->refinedMatches.save(filename);
}

void RefineMatchesStage::loadResults(FeatureMatchingPipeline *pipeline, const std::string &filename) {
	std::cerr << "Loading refined matches from " << filename << std::endl;
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
	std::vector<std::vector<SiftFeature>> features(N);

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

	std::vector<std::vector<size_t>> reordering(N);
	std::vector<std::vector<size_t>> reordering_rev(N);
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
	ofs.open(filename, std::ofstream::out);
	assert(ofs);

	for(size_t i = 0; i < refinedMatches.matchSets.size(); ++i) {
		RefinedMatchSet& set = refinedMatches.matchSets[i];
		ofs << file_name(images[set.imgA].filename) << std::endl << file_name(images[set.imgB].filename) << std::endl;

		ofs << set.matches.size() << std::endl;

		for(auto m: set.matches) {
			assert(m.imgA == set.imgA);
			ofs << reordering_rev[m.imgA][m.featureA] << " ";
		}
		ofs << std::endl;

		for(auto m: set.matches) {
			assert(m.imgB == set.imgB);
			ofs << reordering_rev[m.imgB][m.featureB] << " ";
		}
		ofs << std::endl;
	}
}
