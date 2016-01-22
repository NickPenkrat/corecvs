HEADERS += \
#	cameracalibration/*.h \
        cameracalibration/calibrationCamera.h        \
        cameracalibration/calibrationHelpers.h       \
        cameracalibration/calibrationPhotostation.h  \
        cameracalibration/flatPatternCalibrator.h    \
#        cameracalibration/calibrationFeaturePoint.h \
        cameracalibration/calibrationLocation.h      \
#        cameracalibration/calibrationScene.h         \
        cameracalibration/photoStationCalibrator.h   \


SOURCES += \
#	cameracalibration/*.cpp \
        cameracalibration/calibrationCamera.cpp        \
        cameracalibration/calibrationHelpers.cpp       \
#        cameracalibration/calibrationScene.cpp         \
        cameracalibration/photoStationCalibrator.cpp   \
#        cameracalibration/calibrationFeaturePoint.cpp \
        cameracalibration/calibrationLocation.cpp      \
        cameracalibration/flatPatternCalibrator.cpp    \


CONFIG += c++11
