HEADERS += \
        camerafixture/fixtureCamera.h            \
#	camerafixture/calibrationHelpers.h       \
        camerafixture/cameraFixture.h            \
#	camerafixture/flatPatternCalibrator.h    \
#	camerafixture/calibrationLocation.h      \
        camerafixture/fixtureScene.h             \
#	camerafixture/photoStationCalibrator.h   \
        camerafixture/sceneFeaturePoint.h \
        camerafixture/cameraPrototype.h   \


SOURCES += \
        camerafixture/fixtureCamera.cpp           \
#	camerafixture/calibrationHelpers.cpp      \
        camerafixture/fixtureScene.cpp            \
#	camerafixture/photoStationCalibrator.cpp  \
#	camerafixture/calibrationLocation.cpp     \
#	camerafixture/flatPatternCalibrator.cpp   \
        camerafixture/sceneFeaturePoint.cpp \
        camerafixture/cameraFixture.cpp \
        camerafixture/cameraPrototype.cpp \


CONFIG += c++11
