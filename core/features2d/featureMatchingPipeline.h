#ifndef FEATUREMATCHER_H
#define FEATUREMATCHER_H

#include <vector>
#include <string>
#include <stack>
#include <unordered_map>
#include <chrono>

#include "featureDetectorProvider.h"
#include "descriptorExtractorProvider.h"
#include "matchingPlan.h"
#include "imageKeyPoints.h"
#include "imageMatches.h"

#include "tbb/tbb.h"


class FeatureMatchingPipeline;

class FeatureMatchingPipelineStage {
public:
	virtual void run(FeatureMatchingPipeline *pipeline) = 0;
	virtual void loadResults(FeatureMatchingPipeline *pipeline, const std::string &filename) = 0;
	virtual void saveResults(FeatureMatchingPipeline *pipeline, const std::string &filename) const = 0;
	virtual ~FeatureMatchingPipelineStage() {}
};

class KeyPointDetectionStage : public FeatureMatchingPipelineStage {
public:
	KeyPointDetectionStage(DetectorType type);
	void run(FeatureMatchingPipeline *pipeline);
	void loadResults(FeatureMatchingPipeline *pipeline, const std::string &filename);
	void saveResults(FeatureMatchingPipeline *pipeline, const std::string &filename) const;
	~KeyPointDetectionStage() {};
private:
	DetectorType detectorType;
};

class DescriptorExtractionStage : public FeatureMatchingPipelineStage {
public:
	DescriptorExtractionStage(DescriptorType type);
	void run(FeatureMatchingPipeline *pipeline);
	void loadResults(FeatureMatchingPipeline *pipeline, const std::string &filename);
	void saveResults(FeatureMatchingPipeline *pipeline, const std::string &filename) const;
	~DescriptorExtractionStage() {};
private:
	DescriptorType descriptorType;
};

class MatchingPlanComputationStage : public FeatureMatchingPipelineStage {
public:
	void loadResults(FeatureMatchingPipeline *pipeline, const std::string &filename);
	void saveResults(FeatureMatchingPipeline *pipeline, const std::string &filename) const;
	void run(FeatureMatchingPipeline *pipeline);
	~MatchingPlanComputationStage() {}
};

class MatchingStage : public FeatureMatchingPipelineStage {
public:
	void loadResults(FeatureMatchingPipeline *pipeline, const std::string &filename);
	void saveResults(FeatureMatchingPipeline *pipeline, const std::string &filename) const;
	void run(FeatureMatchingPipeline *pipeline);
	~MatchingStage() {}
	MatchingStage(DescriptorType type, size_t responsesPerPoint = 2);
private:
	DescriptorType descriptorType;
	size_t responsesPerPoint;
};

class RefineMatchesStage : public FeatureMatchingPipelineStage {
public:
	void loadResults(FeatureMatchingPipeline *pipeline, const std::string &filename);
	void saveResults(FeatureMatchingPipeline *pipeline, const std::string &filename) const;
	void run(FeatureMatchingPipeline *pipeline);
	~RefineMatchesStage() {}
	RefineMatchesStage(bool symmetric = true, double scaleThreshold = 1.2);
private:
	bool symmetric;
	double scaleThreshold;
};

class VsfmWriterStage : public FeatureMatchingPipelineStage {
public:
	void loadResults(FeatureMatchingPipeline *pipeline, const std::string &filename) {}
	void saveResults(FeatureMatchingPipeline *pipeline, const std::string &filename) const;
	void run(FeatureMatchingPipeline *pipeline) {}
	VsfmWriterStage(bool sortFeatures);
private:
	bool sortFeatures;
};

class RandIndexStage : public FeatureMatchingPipelineStage {
public:
	void loadResults(FeatureMatchingPipeline *pipeline, const std::string &filename) {}
	void saveResults(FeatureMatchingPipeline *pipeline, const std::string &filename) const;
	void run(FeatureMatchingPipeline *pipeline);
	RandIndexStage();
private:
	std::vector<std::vector<double>> index;
};


class FeatureMatchingPipeline {
public:
	FeatureMatchingPipeline(const std::vector<std::string> &filenames);
	~FeatureMatchingPipeline();
	void run();
	void add(FeatureMatchingPipelineStage* stage, bool run, std::pair<bool, std::string> saveParams = std::make_pair(false, std::string()), std::pair<bool, std::string> loadParams = std::make_pair(false, std::string()));
	void tic(size_t thread_id = ~(size_t)0, bool level = true);
	void toc(const std::string &name, const std::string &evt, size_t thread_id = ~(size_t)0, bool level = true);
	void toc(const std::string &name, const std::string &evt, const size_t curr, const size_t rem, size_t thread_id = ~(size_t)0, bool level = true);

	std::vector<Image> images;
	MatchPlan matchPlan;
	RawMatches rawMatches;
	RefinedMatches refinedMatches;

	DetectorType detectorType;
	DescriptorType descriptorType;

private:
	struct tic_data {
		std::unordered_map<size_t, std::chrono::time_point<std::chrono::high_resolution_clock>> thread_tics;
		std::unordered_map<size_t, size_t> thread_totals, thread_counts;	
	};
	std::stack<tic_data> tics;

	tbb::spin_mutex mutex;
	std::vector<FeatureMatchingPipelineStage*> pipeline;
	std::vector<bool> runEnable;
	std::vector<std::pair<bool, std::string>> saveParams;
	std::vector<std::pair<bool, std::string>> loadParams;
#if 0
	std::stack<std::unordered_map<size_t, std::chrono::time_point<std::chrono::high_resolution_clock>> tics;

	std::stack<std::chrono::time_point<std::chrono::high_resolution_clock>> tics;
	std::vector<size_t> totals, counts;
#endif
	FeatureMatchingPipeline(const FeatureMatchingPipeline&);
};


#endif
