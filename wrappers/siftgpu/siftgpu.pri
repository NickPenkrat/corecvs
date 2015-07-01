CONFIG += c++11

contains(DEFINES, WITH_SIFTGPU) {                    # if it's installed properly with found path for lib

    INCLUDEPATH += $$SIFTGPU_WRAPPER_DIR/SiftGPU/src $$SIFTGPU_WRAPPER_DIR/SiftGPU/include/GL

    HEADERS += \
		$$SIFTGPU_WRAPPER_DIR/siftGpuWrapper.h \
		$$SIFTGPU_WRAPPER_DIR/siftGpuMatcherWrapper.h

    SOURCES += \
		$$SIFTGPU_WRAPPER_DIR/siftGpuWrapper.cpp \
		$$SIFTGPU_WRAPPER_DIR/siftGpuMatcherWrapper.cpp
}
