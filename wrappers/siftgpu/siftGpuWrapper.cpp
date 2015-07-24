#include "siftGpuWrapper.h"

#ifdef WIN32
#include <Windows.h>
#else
#include <dlfcn.h>
#endif
#include <iostream>

#include "runtimeTypeBuffer.h"

#include <GL/glew.h>        // GL_LUMINANCE, ...

SiftGpu::SiftGpu(double filterWidthFactor,
        double orientationFactor,
        double descriptorGridSize,
        int firstOctave,
        int dogLevels,
        double dogThreshold,
        double edgeThreshold,
        int orientationNumber)
: filterWidthFactor(filterWidthFactor), orientationFactor(orientationFactor),
    descriptorGridSize(descriptorGridSize), firstOctave(firstOctave),
    dogLevels(dogLevels), dogThreshold(dogThreshold), edgeThreshold(edgeThreshold),
    orientationNumber(orientationNumber), siftGpu(0)
{
    SiftGPU* (*createNew)(int);
#ifndef WIN32
    void* handle = dlopen("libsiftgpu.so", RTLD_LAZY);
    if(!handle)
    {
        std::cerr << "Failed to open shared lib: " << dlerror() << std::endl;
        exit(0);
    }

    dlerror();

    createNew = (SiftGPU* (*) (int)) dlsym(handle, "CreateNewSiftGPU");
    const char* dlsym_err = dlerror();
    if(dlsym_err)
    {
        std::cerr << "Failed to load fun: " << dlsym_err << std::endl;
        exit(0);
    }
#else
    HINSTANCE hinstLib;

    hinstLib = LoadLibraryA("siftgpu.dll");

    if (!hinstLib)
    {
        std::cerr << "Failed to load shared lib siftgpu.dll" << std::endl;
        exit(0);
    }

    createNew = (SiftGPU* (*) (int)) GetProcAddress(hinstLib, "CreateNewSiftGPU");
    if (!createNew)
    {
        std::cerr << "Failed to load function from siftgpu.dll" << std::endl;
        exit(0);
    }
#endif
    siftGpu = createNew(1);
    const char *argv[] = {"-v", "0", "-fo", "-1", "-da", "-tc2", "7680", "-nomc"};
    int argc = CORE_COUNT_OF(argv);

    siftGpu->ParseParam(argc, const_cast<char**>(argv));
    int support = siftGpu->CreateContextGL();

    if (support != SiftGPU::SIFTGPU_FULL_SUPPORTED)
    {
        std::cerr << support << " siftgpu is unsupported" << std::endl;
        exit(0);
    }
}

void SiftGpu::computeImpl(RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints, RuntimeTypeBuffer &descriptors)
{
    (*this)(image, keyPoints, descriptors, true, true);
}

void SiftGpu::detectImpl(RuntimeTypeBuffer &image, std::vector<KeyPoint> &keyPoints)
{
    RuntimeTypeBuffer buffer;
    (*this)(image, keyPoints,buffer);
}

int SiftGpu::descriptorSize() const
{
    return 128;
}

int SiftGpu::descriptorType() const
{
    return CV_32F;
}

void SiftGpu::operator()(RuntimeTypeBuffer &img, std::vector<KeyPoint>& keypoints) const
{
    RuntimeTypeBuffer buffer;
    (*this)(img, keypoints, buffer);
}

SiftGPU::SiftKeypoint SiftGpu::convert(const KeyPoint &k)
{
    //TODO: check what orientation and size really represent
    SiftGPU::SiftKeypoint kp =
    { k.x, k.y, k.size, k.angle };
    return kp;
}

KeyPoint SiftGpu::convert(const SiftGPU::SiftKeypoint &k)
{
    return KeyPoint(k.x, k.y, k.s, k.o);
}

void SiftGpu::operator()(RuntimeTypeBuffer &image, std::vector<KeyPoint> &keypoints, RuntimeTypeBuffer &descriptors, bool computeDescriptors, bool useProvidedKeypoints) const
{
    if (image.getType() != BufferType::U8 || !image.isValid())
    {
        std::cerr << __LINE__ << "Invalid image type" << std::endl;
    }

    std::vector<SiftGPU::SiftKeypoint> keypoints_sgpu;
    if (useProvidedKeypoints)
    {
        keypoints_sgpu.reserve(keypoints.size());
        for (std::vector<KeyPoint>::iterator kp = keypoints.begin(); kp != keypoints.end(); ++kp)
            keypoints_sgpu.push_back(SiftGpu::convert(*kp));
        siftGpu->SetKeypointList((int)keypoints_sgpu.size(), &keypoints_sgpu[0]);
        siftGpu->RunSIFT((int)image.getCols(), (int)image.getRows(), image.row<unsigned char>(0), GL_LUMINANCE, GL_UNSIGNED_BYTE);
    }
    else
    {
        siftGpu->RunSIFT((int)image.getCols(), (int)image.getRows(), image.row<unsigned char>(0), GL_LUMINANCE, GL_UNSIGNED_BYTE);
        int num = siftGpu->GetFeatureNum();
        keypoints_sgpu.resize(num);
        siftGpu->GetFeatureVector(&keypoints_sgpu[0], 0);
        keypoints.reserve(num);
        for (std::vector<SiftGPU::SiftKeypoint>::iterator kp = keypoints_sgpu.begin(); kp != keypoints_sgpu.end(); ++kp)
            keypoints.push_back(SiftGpu::convert(*kp));
    }

    if (computeDescriptors)
    {
        descriptors = RuntimeTypeBuffer(keypoints.size(), descriptorSize(), BufferType::F32);
        siftGpu->GetFeatureVector(0, descriptors.row<float>(0));
    }
}

SiftGpu::~SiftGpu()
{
#ifdef WIN32
    delete siftGpu;     // windows requires to "delete" the allocated object inside siftGpu.dll instead of free-ing it
#else
    free(siftGpu);       // linux: valgrid said that free is better
#endif
}

#define SWITCH_TYPE(str, expr) \
    if (type == #str) \
    { \
        expr; \
    }

DescriptorExtractor* SiftGpuDescriptorExtractorProvider::getDescriptorExtractor(const DescriptorType &type)
{
    SWITCH_TYPE(SIFTGPU,
       return new SiftGpu();)
    return 0;
}

bool SiftGpuDescriptorExtractorProvider::provides(const DescriptorType &type)
{
    SWITCH_TYPE(SIFTGPU, return true;);
    return false;
}


FeatureDetector* SiftGpuFeatureDetectorProvider::getFeatureDetector(const DetectorType &type)
{
    SWITCH_TYPE(SIFTGPU,
            return new SiftGpu();)
    return 0;
}

bool SiftGpuFeatureDetectorProvider::provides(const DetectorType &type)
{
    SWITCH_TYPE(SIFTGPU, return true;);
    return false;
}

void init_siftgpu_detector_provider()
{
    FeatureDetectorProvider::getInstance().add(new SiftGpuFeatureDetectorProvider);
}
void init_siftgpu_descriptor_provider()
{
    DescriptorExtractorProvider::getInstance().add(new SiftGpuDescriptorExtractorProvider);
}

#undef SWITCH_TYPE
