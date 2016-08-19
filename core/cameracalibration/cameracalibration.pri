HEADERS += \
        cameracalibration/calibrationCamera.h        \
        cameracalibration/calibrationHelpers.h       \
        cameracalibration/calibrationPhotostation.h  \
        cameracalibration/flatPatternCalibrator.h    \
        cameracalibration/calibrationLocation.h      \
        cameracalibration/photoStationCalibrator.h \
    $$PWD/projectionModels.h


SOURCES += \
        cameracalibration/calibrationCamera.cpp        \
        cameracalibration/calibrationHelpers.cpp       \
        cameracalibration/photoStationCalibrator.cpp   \
        cameracalibration/calibrationLocation.cpp      \
        cameracalibration/flatPatternCalibrator.cpp \
    $$PWD/projectionModels.cpp

