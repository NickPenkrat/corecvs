#ifndef SIFTGPUMATCHERWRAPPER_H
#define SIFTGPUMATCHERWRAPPER_H

#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "SiftGPU/SiftGPU.h"


class SiftGpuMatcher : public cv::DescriptorMatcher {
	public:
		SiftGpuMatcher();
		SiftGpuMatcher(const SiftGpuMatcher &c);
		virtual ~SiftGpuMatcher();

		virtual bool isMaskSupported() const { return false; }

		virtual cv::Ptr<cv::DescriptorMatcher> clone( bool emptyTrainData=false ) const;

//		cv::AlgorithmInfo* info() const
	protected:
		virtual void knnMatchImpl( const cv::Mat& queryDescriptors, std::vector<std::vector<cv::DMatch> >& matches, int k,
				const std::vector<cv::Mat>& masks=std::vector<cv::Mat>(), bool compactResult=false );
		virtual void radiusMatchImpl( const cv::Mat& queryDescriptors, std::vector<std::vector<cv::DMatch> >& matches, float maxDistance,
				const std::vector<cv::Mat>& masks=std::vector<cv::Mat>(), bool compactResult=false );
	private:
		SiftMatchGPU* initSiftMatchGpu(int count = 8192);
		SiftMatchGPU* siftMatchGpu;
};


#endif
