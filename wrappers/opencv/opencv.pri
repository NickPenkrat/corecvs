isEmpty(OPENCV_WRAPPER_DIR) {
    message(Incorrect usage of opencv.pri with empty OPENCV_WRAPPER_DIR. OpenCV is switched off!)
} else {
    include(opencvLibs.pri)
}

contains(DEFINES, WITH_OPENCV) {                    # if it's installed properly with found path for lib

    INCLUDEPATH += $$OPENCV_WRAPPER_DIR

    HEADERS += \
		$$OPENCV_WRAPPER_DIR/featureDetectorCV.h \
		$$OPENCV_WRAPPER_DIR/KLTFlow.h \
		$$OPENCV_WRAPPER_DIR/openCvDescriptorExtractorWrapper.h \
		$$OPENCV_WRAPPER_DIR/openCvDescriptorMatcherWrapper.h \
		$$OPENCV_WRAPPER_DIR/openCvFeatureDetectorWrapper.h \
		$$OPENCV_WRAPPER_DIR/openCvFileReader.h \
		$$OPENCV_WRAPPER_DIR/openCvKeyPointsWrapper.h \
                $$OPENCV_WRAPPER_DIR/openCVTools.h \
		$$OPENCV_WRAPPER_DIR/semiGlobalBlockMatching.h \
		$$OPENCV_WRAPPER_DIR/openCvCheckerboardDetector.h \
                $$OPENCV_WRAPPER_DIR/openCvImageRemapper.h \
                $$OPENCV_WRAPPER_DIR/openCvDetectAndExtractWrapper.h \

    contains(DEFINES, WITH_OPENCV_GPU) { 
        HEADERS += \      
                $$OPENCV_WRAPPER_DIR/openCvGPUDescriptorExtractorWrapper.h \
                $$OPENCV_WRAPPER_DIR/openCvGPUDescriptorMatcherWrapper.h \
                $$OPENCV_WRAPPER_DIR/openCvGPUFeatureDetectorWrapper.h \
                $$OPENCV_WRAPPER_DIR/openCvGPUDetectAndMatchWrapper.h \
                $$OPENCV_WRAPPER_DIR/openCvGPUDetectAndExtractWrapper.h \
    }

    SOURCES += \
		$$OPENCV_WRAPPER_DIR/openCvFeatureDetectorWrapper.cpp \
		$$OPENCV_WRAPPER_DIR/featureDetectorCV.cpp \
                $$OPENCV_WRAPPER_DIR/openCVTools.cpp \
		$$OPENCV_WRAPPER_DIR/openCvDescriptorMatcherWrapper.cpp \
		$$OPENCV_WRAPPER_DIR/openCvFileReader.cpp \
		$$OPENCV_WRAPPER_DIR/KLTFlow.cpp \
		$$OPENCV_WRAPPER_DIR/openCvKeyPointsWrapper.cpp \
		$$OPENCV_WRAPPER_DIR/openCvDescriptorExtractorWrapper.cpp \
		$$OPENCV_WRAPPER_DIR/semiGlobalBlockMatching.cpp \
		$$OPENCV_WRAPPER_DIR/openCvCheckerboardDetector.cpp \
                $$OPENCV_WRAPPER_DIR/openCvImageRemapper.cpp \
                $$OPENCV_WRAPPER_DIR/openCvDetectAndExtractWrapper.cpp \

    contains(DEFINES, WITH_OPENCV_GPU) { 
        SOURCES += \      
                $$OPENCV_WRAPPER_DIR/openCvGPUDescriptorExtractorWrapper.cpp \
                $$OPENCV_WRAPPER_DIR/openCvGPUDescriptorMatcherWrapper.cpp \
                $$OPENCV_WRAPPER_DIR/openCvGPUFeatureDetectorWrapper.cpp \
                $$OPENCV_WRAPPER_DIR/openCvGPUDetectAndMatchWrapper.cpp \
                $$OPENCV_WRAPPER_DIR/openCvGPUDetectAndExtractWrapper.cpp \
    }

    # Face Detection
    #
    INCLUDEPATH += $$OPENCV_WRAPPER_DIR/faceDetect

    HEADERS     += $$OPENCV_WRAPPER_DIR/faceDetect/faceDetect.h
    SOURCES     += $$OPENCV_WRAPPER_DIR/faceDetect/faceDetect.cpp

    HEADERS     += $$PWD/xml/generated/*.h
    SOURCES     += $$PWD/xml/generated/*.cpp

    with_meshflow {
        HEADERS += \
            $$PWD/moc-flow/openCVMovingObjectFlow.h \
            $$PWD/moc-flow/meshFlowAlgo.h

        SOURCES += \
            $$PWD/moc-flow/openCVMovingObjectFlow.cpp \
            $$PWD/moc-flow/meshFlowAlgo.cpp
    }

}

OTHER_FILES += $$PWD/xml/*.xml
OTHER_FILES += $$PWD/../../tools/generator/regen-opencv.sh

INCLUDEPATH += $$PWD/xml/generated

INCLUDEPATH += $$PWD/moc-flow



