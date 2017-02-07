HEADERS += \
        cameracalibration/calibrationCamera.h        \
        cameracalibration/flatPatternCalibrator.h    \
        cameracalibration/calibrationLocation.h      \
        cameracalibration/projectionModels.h         \
        cameracalibration/cameraConstraints.h \
    $$PWD/calibrationDrawHelpers.h \
    $$PWD/pinholeCameraIntrinsics.h


SOURCES += \
        cameracalibration/calibrationCamera.cpp        \
        cameracalibration/calibrationLocation.cpp      \
        cameracalibration/flatPatternCalibrator.cpp    \
        cameracalibration/projectionModels.cpp         \
        cameracalibration/cameraConstraints.cpp \
    $$PWD/calibrationDrawHelpers.cpp \
    $$PWD/pinholeCameraIntrinsics.cpp

