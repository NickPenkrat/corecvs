HEADERS += \
        features2d/algoBase.h                           \
        features2d/bufferReaderProvider.h               \
        features2d/descriptorExtractorProvider.h        \
        features2d/descriptorMatcherProvider.cpp        \
        features2d/descriptorMatcherProvider.h          \
        features2d/detectAndExtractProvider.h           \
        features2d/detectExtractAndMatchProvider.cpp    \
        features2d/detectExtractAndMatchProvider.h      \
        features2d/featureDetectorProvider.h            \
        features2d/featureMatchingPipeline.cpp          \
        features2d/featureMatchingPipeline.h            \
        features2d/imageKeyPoints.h                     \
        features2d/imageMatches.cpp                     \
        features2d/imageMatches.h                       \
        features2d/matchingPlan.cpp                     \
        features2d/matchingPlan.h                       \
        features2d/trackPainter.h                       \
        features2d/vsfmIo.h




SOURCES += \
features2d/bufferReaderProvider.cpp             \
features2d/descriptorExtractorProvider.cpp      \
features2d/descriptorMatcherProvider.cpp        \
features2d/detectAndExtractProvider.cpp         \
features2d/detectExtractAndMatchProvider.cpp    \
features2d/featureDetectorProvider.cpp          \
features2d/featureMatchingPipeline.cpp          \
features2d/imageKeyPoints.cpp                   \
features2d/imageMatches.cpp                     \
features2d/matchingPlan.cpp                     \
features2d/trackPainter.cpp                     \
features2d/vsfmIo.cpp
