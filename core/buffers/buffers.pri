HEADERS += \ 
    buffers/memory/memoryBlock.h \
    buffers/abstractBuffer.h \
    buffers/abstractContiniousBuffer.h \
    buffers/abstractKernel.h \
    buffers/bufferFactory.h \
    buffers/disparityBuffer.h \
    buffers/displacementBuffer.h \
    buffers/g8Buffer.h \
    buffers/g12Buffer.h \
    buffers/commonMappers.h \
    buffers/integralBuffer.h \
    buffers/derivativeBuffer.h \
    buffers/mipmapPyramid.h \
    buffers/flow/flowBuffer.h \
    buffers/flow/sixDBuffer.h \
    buffers/flow/floatFlowBuffer.h \
    buffers/flow/punchedBufferOperations.h \
    buffers/flow/flowVector.h \
    buffers/flow/depthBuffer.h \
    buffers/histogram/histogram.h \
    buffers/kernels/gaussian.h \
    buffers/kernels/sobel.h \
    buffers/kernels/threshold.h \
    buffers/kernels/arithmetic.h \
    buffers/kernels/copyKernel.h \
    buffers/kernels/fastkernel/fastKernel.h \
    buffers/kernels/fastkernel/readers.h \
    buffers/kernels/fastkernel/baseKernel.h \
    buffers/kernels/fastkernel/baseAlgebra.h \
    buffers/kernels/fastkernel/vectorAlgebra.h \
    buffers/kernels/fastkernel/scalarAlgebra.h \
    buffers/kernels/blurProcessor.h \
    buffers/kernels/spatialGradient.h \
    buffers/morphological/morphological.h \
    buffers/rgb24/rgbColor.h \
    buffers/rgb24/rgb24Buffer.h \
    buffers/rgb24/hardcodeFont.h \
    buffers/rgb24/hersheyVectorFont.h \
    buffers/rgb24/abstractPainter.h \
    buffers/voxels/voxelBuffer.h \
    buffers/fixeddisp/fixedPointDisplace.h \
    buffers/interpolator.h \
    buffers/g12Buffer3d.h \
    buffers/buffer3d.h \
    buffers/transformationCache.h \


SOURCES += \
    buffers/memory/memoryBlock.cpp \
    buffers/abstractBuffer.cpp \
    buffers/bufferFactory.cpp \
    buffers/disparityBuffer.cpp \
    buffers/displacementBuffer.cpp \
    buffers/g8Buffer.cpp \
    buffers/g12Buffer.cpp \
    buffers/commonMappers.cpp \
    buffers/mipmapPyramid.cpp \
    buffers/derivativeBuffer.cpp \
    buffers/flow/flowBuffer.cpp \
    buffers/flow/sixDBuffer.cpp \
    buffers/flow/floatFlowBuffer.cpp \
    buffers/flow/depthBuffer.cpp \
    buffers/histogram/histogram.cpp \
    buffers/kernels/gaussian.cpp \
    buffers/kernels/sobel.cpp \
    buffers/kernels/threshold.cpp \
    buffers/kernels/blurProcessor.cpp \
    buffers/kernels/spatialGradient.cpp \
    buffers/morphological/morphological.cpp \
    buffers/rgb24/rgb24Buffer.cpp \
    buffers/rgb24/rgbColor.cpp \
    buffers/rgb24/hardcodeFont.cpp \
    buffers/rgb24/hersheyVectorFont.cpp \    
    buffers/rgb24/abstractPainter.cpp \
    buffers/g12Buffer3d.cpp \
    buffers/buffer3d.cpp \
    buffers/transformationCache.cpp \


