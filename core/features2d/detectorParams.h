#ifndef DETECTORPARAMS_H
#define DETECTORPARAMS_H

#include <opencv2/features2d/features2d.hpp>

struct DetectorsParams {
	struct SURF {
		double hessianThreshold;
		int octaves;
		int octaveLayers;
		bool extended;
		bool upright;


		SURF(
				double hessianThreshold = 400.0,
				int octaves = 4,
				int octaveLayers = 2,
				bool extended = false,
				bool upright = false) 
			: hessianThreshold(hessianThreshold),
			octaves(octaves),
			octaveLayers(octaveLayers),
			extended(extended),
			upright(upright) {
			}
	};

	struct SIFT {
		double contrastThreshold;
		double edgeThreshold;
		double sigma;
		int nOctaveLayers;

		SIFT(
				double contrastThreshold = 0.04,
				double edgeThreshold = 10.0,
				double sigma = 1.6,
				int nOctaveLayers = 3) :
			contrastThreshold(contrastThreshold),
			edgeThreshold(edgeThreshold),
			sigma(sigma),
			nOctaveLayers(nOctaveLayers) {
			}
	};

	struct STAR {
		int maxSize;
		int responseThreshold;
		int lineThresholdProjected;
		int lineThresholdBinarized;
		int supressNonmaxSize;

		STAR(
				int maxSize = 16,
				int responseThreshold = 30,
				int lineThresholdProjected = 10,
				int lineThresholdBinarized = 8,
				int supressNonmaxSize = 5) :
			maxSize(maxSize),
			responseThreshold(responseThreshold),
			lineThresholdProjected(lineThresholdProjected),
			lineThresholdBinarized(lineThresholdBinarized),
			supressNonmaxSize(supressNonmaxSize) {
			}
	};

	struct FAST {
		int threshold;
		bool nonmaxSuppression;
		int type;

		FAST(
				int threshold = 1,
				bool nonmaxSuppression = true,
				int type = 2) ://cv::FastFeatureDetector::TYPE_9_16) :
			threshold(threshold), nonmaxSuppression(nonmaxSuppression), type(type) {
			}
		
	};

	struct ORB {
		double scaleFactor;
		int nLevels;
		int edgeThreshold;
		int firstLevel;
		int WTA_K;
		int scoreType;
		int patchSize;

		ORB(double scaleFactor = 1.2, int nLevels = 8, int edgeThreshold = 31,
				int firstLevel = 0, int WTA_K = 2, int scoreType = 0, //cv::ORB::HARRIS_SCORE, 
				int patchSize = 31):
			scaleFactor(scaleFactor), nLevels(nLevels), edgeThreshold(edgeThreshold), firstLevel(firstLevel),
			WTA_K(WTA_K), scoreType(scoreType), patchSize(patchSize) {
			}
	};

	struct BRISK {
		int thresh;
		int octaves;
		double patternScale;

		BRISK(int thresh = 30, int octaves = 3, double patternScale = 1.0) :
			thresh(thresh), octaves(octaves), patternScale(patternScale) {
			}
	};

	SURF  surfParams;
	SIFT  siftParams;
	STAR  starParams;
	FAST  fastParams;
	BRISK briskParams;
	ORB   orbParams;
};

#endif
