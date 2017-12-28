HEADERS += \
        cameracalibration/flatPatternCalibrator.h    \
        cameracalibration/calibrationLocation.h      \
        cameracalibration/projectionModels.h         \
        cameracalibration/cameraConstraints.h        \
        cameracalibration/calibrationDrawHelpers.h   \
        cameracalibration/pinholeCameraIntrinsics.h  \
    $$PWD/cameraModel.h


SOURCES += \
        cameracalibration/calibrationLocation.cpp      \
        cameracalibration/flatPatternCalibrator.cpp    \
        cameracalibration/projectionModels.cpp         \
        cameracalibration/cameraConstraints.cpp        \
        cameracalibration/calibrationDrawHelpers.cpp   \
        cameracalibration/pinholeCameraIntrinsics.cpp  \
    $$PWD/cameraModel.cpp

