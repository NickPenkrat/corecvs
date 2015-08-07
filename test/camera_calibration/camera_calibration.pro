# try use global config 
exists(../../../../config.pri) {
    ROOT_DIR=../../../..
    #message(Using global config)
} else { 
    message(Using local config)
    ROOT_DIR=../..
}
ROOT_DIR=$$PWD/$$ROOT_DIR
include($$ROOT_DIR/config.pri)

TEMPLATE=app
TARGET  = test_camera_calibration
CONFIG += CONSOLE
QT += core
QT -= gui
QT -= xml

UTILSDIR = $$ROOT_DIR/src/open/utils
include($$UTILSDIR/utils.pri)

SOURCES += camera_calibration.cpp \
           flatPatternCalibrator.cpp \
           photoStationCalibrator.cpp

INCLUDES += typesafeBitmaskEnums.h \
            calibration_structs.h \
            flatPatternCalibration.h \
            photoStationCalibrator.h
