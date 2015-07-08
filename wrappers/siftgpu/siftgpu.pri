contains(DEFINES, WITH_SIFTGPU) {                    # if it's installed properly with found path for lib

    INCLUDEPATH += \
	$$SIFTGPU_WRAPPER_DIR/SiftGPU/src \
	$$SIFTGPU_WRAPPER_DIR/SiftGPU/include/

    HEADERS += \
	$$SIFTGPU_WRAPPER_DIR/siftGpuWrapper.h \
	$$SIFTGPU_WRAPPER_DIR/siftGpuMatcherWrapper.h

    SOURCES += \
	$$SIFTGPU_WRAPPER_DIR/siftGpuWrapper.cpp \
	$$SIFTGPU_WRAPPER_DIR/siftGpuMatcherWrapper.cpp

    #QMAKE_LFLAGS += -Wl,-u,init_siftgpu_detector_provider,-u,init_siftgpu_descriptor_provider
}
