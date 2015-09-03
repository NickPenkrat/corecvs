# try use global config 
exists(../../../../config.pri) {
    ROOT_DIR=../../../..
    #message(Using global config1)
} else { 
    message(Using local config)
    ROOT_DIR=../..
}
ROOT_DIR=$$PWD/$$ROOT_DIR
include($$ROOT_DIR/config.pri)

TEMPLATE=app
TARGET  = test_calibration_job_estimate_distortion
CONFIG += CONSOLE

UTILSDIR = $$ROOT_DIR/src/open/utils
include($$UTILSDIR/utils.pri)

SOURCES += calibration_job_estimate_distortion.cpp

