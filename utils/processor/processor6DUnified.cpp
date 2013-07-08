/*
 * processor6DUnified.cpp
 *
 *  Created on: Oct 5, 2012
 *      Author: alexander
 */

#include "preciseTimer.h"

#include "processor6DUnified.h"
#include "viFlowStatisticsDescriptor.h"

#include "cortageListSorter.h"
#include "cortageStripeMatcher.h"

#ifdef WITH_OPENCV
#include "semiGlobalBlockMatching.h"
#endif

#ifdef WITH_HARDWARE
#include "hardwareCortegeListSorter.h"
#endif

#ifdef WITH_OPENCL
#include "../../../opencl/OpenCLAlgo.h"
#include "../../../opencl/OpenCLKernelStructures.h"
using namespace cl;
#include "featuringParameters.h"
#endif

#ifdef WITH_LIBELAS
#include "libViso2Provider.h"
#endif

void Processor6DUnified::initOpenCLProcessor()
{
#ifdef WITH_OPENCL
    mFirstPair = true;

    if (openCLProcessor != NULL)
        return;

    /* Creating sys parameters */
    OpenCLBufferParamsList bufsParams;
    openCLKernelParams = new OpenCLKernelParams();
    openCLKernelParams->mSys.init(bufsParams);

    /* Preparing devices */
    OpenCLDeviceIterator deviceIterator;

    cout << "Processor6DUnified::initOpenCLProcessor(): OpenCL is On. " << endl
         << "Enumerating devices:" << endl;

    while (const OpenCLDevice* pDevice = deviceIterator.GetNextDevice())
    {
        cout << "   " << pDevice->uniqName() << endl;

        if (pDevice->isEmulator())
        {
            openCLProcessor = new OpenCLAlgo(pDevice, bufsParams, openCLKernelParams->mSys);

            if (!openCLProcessor->isCreated()) {
                SYNC_PRINT(("Processor6DUnified::initOpenCLProcessor(): Can't create emulator device\n"));
            }
        }
        else if (pDevice->deviceInfo()->eDevType == cl::gpu && openCLProcessorHard == NULL)     // take the first GPU device
        {
            openCLProcessorHard = new OpenCLAlgo(pDevice, bufsParams, openCLKernelParams->mSys);

            if (!openCLProcessorHard->isCreated()) {
                SYNC_PRINT(("Processor6DUnified::initOpenCLProcessor(): Can't create hardware device\n"));
            }
        }
        delete pDevice;
    }
#endif // WITH_OPENCL
}

Processor6DUnified::Processor6DUnified() :
	openCLProcessor(NULL),
	openCLProcessorHard(NULL),
	openCLKernelParams(NULL),
    mFirstPair(false)
{
	initOpenCLProcessor();
}

void Processor6DUnified::setOpenCVBMParameters(const OpenCVBMParameters &openCVBMParameters)
{
    mOpenCVBMParameters = openCVBMParameters;
}

void Processor6DUnified::setOpenCVSGMParameters(const OpenCVSGMParameters &openCVSGMParameters)
{
    mOpenCVSGMParameters = openCVSGMParameters;
}

int Processor6DUnified::setFrameG12(FrameNames frameType, G12Buffer *frame)
{
#ifdef WITH_OPENCL
	if (mProviderParameters.stereoProvider() == StereoProvider::VI_TECH_OPENCL ||
	    mProviderParameters.stereoProvider() == StereoProvider::VI_TECH_OPENCL_EMU)
	{
		OpenCLAlgo *currentProcessor = openCLProcessor;
		if (mProviderParameters.stereoProvider() == StereoProvider::VI_TECH_OPENCL && openCLProcessorHard != NULL) {
			currentProcessor = openCLProcessorHard;
		}

		/*Parameters for featuring*/
		FeaturingParameters featuringParameters(
				FeaturingAlgorithm::STATIC_FEATURING,
				mLinearSignatureParameters,
				mAdaptiveParameters,
				PcaEngineParameters()
		);

		if (!openCLKernelParams->SetFeaturingParameters(featuringParameters))
        {
			SYNC_PRINT(("Processor6DUnified::setFrameG12(): Error setting up parameters\n"));
			return -1;
		}

		RectCortegeList **featuredCorteges;
//		FeaturingBuffer **featureBuffer;
//		FeatureVectorProducer *producer;
        int oclIdxFrame64;

		switch (frameType) {
		case Processor6D::FRAME_LEFT_ID:
			featuredCorteges = &mCurrentSSLeft;
			mFrameLeft       = frame;
          //featureBuffer    = &mCurrentFBLeft;
          //producer         = &leftProducer;
            oclIdxFrame64    = esbiFrame64_1;
			break;
		case Processor6D::FRAME_RIGHT_ID:
			featuredCorteges = &mCurrentSSRight;
			mFrameRight      = frame;
          //featureBuffer    = &mCurrentFBRight;
          //producer         = &rightProducer;
            oclIdxFrame64    = esbiFrame64_0;
			break;
		default:
			return -1;
		}

		OpenCLBuffer inputImage(frame);

		/* Running executer for FeatureStatic procedure */
        EAlgoOp eOp = currentProcessor->isGPU() ? cl::eAlgoFeaturingStatic2 : cl::eAlgoFeaturingStatic;

		bool res = currentProcessor->execFeaStat(eOp, *openCLKernelParams, &inputImage, oclIdxFrame64);
		if (!res) {
			SYNC_PRINT(("Processor6DUnified::setFrameG12(): Executer failed\n"));
			return -1;
		}

		OpenCLBuffer resImage;
		if (!currentProcessor->readBuffer(resImage, oclIdxFrame64, *openCLKernelParams)) {
			SYNC_PRINT(("Processor6DUnified::setFrameG12(): reading buffer failed\n"));
			return -1;
		}

		SYNC_PRINT(("Result form executer has size [%dx%d]\n", resImage.width(), resImage.height()));

		/* Converting result */
	    cuint offsXY     = resImage.offsetXY();
	    cuint stride     = resImage.stride();
	  //cuint strideHalf = stride / 2;
	  //cuint width      = resImage.width();

	    RectCortegeList *toReturn = new RectCortegeList(frame->h, frame->w, offsXY, offsXY);

	    for (uint i = 0; i < resImage.height(); i++) {
	    	const CortegeEx* runnerIn = ((CortegeEx*)resImage.data() + stride * i);
   	        Cortege* runnerOut = &(toReturn->element(i, 0));

		    for (uint j = resImage.width(); j > 0; j--) {
		    	*runnerOut++ = (Cortege) *runnerIn++;
		    }
	    }

		delete *featuredCorteges;
		*featuredCorteges = toReturn;

//		(*featuredCorteges)->fillWith(Cortege(0, Vector2d16(1,1)));

        return 0;
	}
    else
#endif // WITH_OPENCL
    {
		return Processor6DImpl::setFrameG12(frameType, frame);
	}
}

int Processor6DUnified::endFrame()
{
    FlowBuffer *stereoOpenCV = NULL;

    PreciseTimer timer;

    if (mProviderParameters.stereoProvider() == StereoProvider::OPENCV_SGM)
    {
        timer = PreciseTimer::currentTime();
#ifdef WITH_OPENCV
        SGBMOpenCV sgbm;
        stereoOpenCV = sgbm.getStereoSGBM(mFrameLeft,mFrameRight, mOpenCVSGMParameters);
#endif
        Processor6DImpl::saveStatTime(ViFlowStatisticsDescriptor::SGM_TIME, timer.usecsToNow());
    }
    else if (mProviderParameters.stereoProvider() == StereoProvider::OPENCV_BM)
    {
        timer = PreciseTimer::currentTime();
#ifdef WITH_OPENCV
        BMOpenCV bm;
        stereoOpenCV = bm.getStereoBM(mFrameLeft,mFrameRight, mOpenCVBMParameters);
#endif
        Processor6DImpl::saveStatTime(ViFlowStatisticsDescriptor::BLOCKMATCH_TIME, timer.usecsToNow());
    }
    else if (mProviderParameters.stereoProvider() == StereoProvider::VI_TECHNOLOGY_HARDWARE)
    {
#ifdef WITH_HARDWARE
    	MatchingParameters hardwareParameters = mMatchingParameters;
    	if ((hardwareParameters.stripeHeight() % 2) != 0) {
    		hardwareParameters.setStripeHeight(hardwareParameters.stripeHeight() + 1);
    	}
        // TODO: Fix this logic ASAP
        SYNC_PRINT(("We are starting hardware call logic\n"));

        SYNC_PRINT(("   mCurrentSSRight is %p\n", mCurrentSSRight));
        SYNC_PRINT(("   stripeHeight is %d\n", hardwareParameters.stripeHeight()));

        // Sorting within tiles for stereo matching
        HardwareCortegeListSorter().sortStripes(mCurrentSSRight, hardwareParameters.stripeHeight());
        HardwareCortegeListSorter().sortStripes(mCurrentSSLeft , hardwareParameters.stripeHeight());

        //saveStatTime(mStats->PRE_SORTING_TIME, startEl.usecsToNow());

        CortegeStripeMatcher matcherTiles;
        //startEl = PreciseTimer::currentTime();

        // Matching within tiles to get stereo match
        stereoOpenCV = matcherTiles.doStereoStripeMatch(mCurrentSSRight,mCurrentSSLeft, hardwareParameters);
        //saveStatTime(mStats->STEREO_MATCHING_TIME, startEl.usecsToNow());
#endif
    }
    else if (mProviderParameters.stereoProvider() == StereoProvider::VI_TECH_OPENCL ||
             mProviderParameters.stereoProvider() == StereoProvider::VI_TECH_OPENCL_EMU)
    {
#ifdef WITH_OPENCL
		OpenCLAlgo *currentProcessor = openCLProcessor;
		if (mProviderParameters.stereoProvider() == StereoProvider::VI_TECH_OPENCL && openCLProcessorHard != NULL) {
			currentProcessor = openCLProcessorHard;
		}

		currentProcessor->execMain3ViPostFeaturing(*openCLKernelParams, mFirstPair, openCLProcessor);
        mFirstPair = false;

        OpenCLBuffer resImage;
        openCLProcessor->readBuffer(resImage, esbiFrame32s_1, *openCLKernelParams);
/*
        if (currentProcessor->isGPU() || currentProcessor->isEmul())
            openCLProcessor->readBuffer(resImage, esbiFrame32s_1, *openCLKernelParams);
        else
		    currentProcessor->readBuffer(resImage, esbiFrame32s_1, *openCLKernelParams);
*/
        stereoOpenCV = new FlowBuffer(resImage.height(), resImage.width(), false, resImage.stride());
        memcpy(stereoOpenCV->data, resImage.data(), stereoOpenCV->sizeInBytes());
#endif
    }
    else if (mProviderParameters.stereoProvider() == StereoProvider::LIB_VISO_2)
    {
#ifdef WITH_LIBELAS
        stereoOpenCV = LibViso2Provider::getStereo(mFrameRight, mFrameLeft);
#endif
    }

    Processor6DImpl::setDisparityBufferS16(FRAME_LEFT_ID, stereoOpenCV);

    int result = Processor6DImpl::endFrame();

    if (stereoOpenCV != NULL) {
        delete_safe(stereoOpenCV);
    }

    return result;
}

Processor6DUnified::~Processor6DUnified()
{
#ifdef WITH_OPENCL
	delete_safe(openCLProcessorHard);
	delete_safe(openCLProcessor);
	delete_safe(openCLKernelParams);
#endif
}
