#ifndef FEATUREMATCHER_H
#define FEATUREMATCHER_H

#include <vector>
#include <string>

#include "detectorParams.h"
#include "featureDetectorProvider.h"
#include "descriptorExtractorProvider.h"
#include "matchingPlan.h"
#include "imageKeyPoints.h"
#include "imageMatches.h"


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
	KeyPointDetectionStage(DetectorType type, DetectorsParams params);
	void run(FeatureMatchingPipeline *pipeline);
	void loadResults(FeatureMatchingPipeline *pipeline, const std::string &filename);
	void saveResults(FeatureMatchingPipeline *pipeline, const std::string &filename) const;
	~KeyPointDetectionStage() {};
private:
	DetectorType detectorType;
	DetectorsParams detectorsParams;
};

class DescriptorExtractionStage : public FeatureMatchingPipelineStage {
public:
	DescriptorExtractionStage(DescriptorType type, DetectorsParams params);
	void run(FeatureMatchingPipeline *pipeline);
	void loadResults(FeatureMatchingPipeline *pipeline, const std::string &filename);
	void saveResults(FeatureMatchingPipeline *pipeline, const std::string &filename) const;
	~DescriptorExtractionStage() {};
private:
	DescriptorType descriptorType;
	DetectorsParams detectorsParams;
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

class FeatureMatchingPipeline {
public:
	FeatureMatchingPipeline(const std::vector<std::string> &filenames);
	~FeatureMatchingPipeline();
	void run();
	void add(FeatureMatchingPipelineStage* stage, bool run, std::pair<bool, std::string> saveParams = std::make_pair(false, std::string()), std::pair<bool, std::string> loadParams = std::make_pair(false, std::string()));

	std::vector<Image> images;
	MatchPlan matchPlan;
	RawMatches rawMatches;
	RefinedMatches refinedMatches;

	DetectorType detectorType;
	DescriptorType descriptorType;
	DetectorsParams detectorsParams;

private:
	std::vector<FeatureMatchingPipelineStage*> pipeline;
	std::vector<bool> runEnable;
	std::vector<std::pair<bool, std::string>> saveParams;
	std::vector<std::pair<bool, std::string>> loadParams;
	FeatureMatchingPipeline(const FeatureMatchingPipeline&);
};


#endif
