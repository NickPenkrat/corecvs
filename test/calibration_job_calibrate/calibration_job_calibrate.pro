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

TEMPLATE = app
TARGET   = test_calibration_job_calibrate
CONFIG  += CONSOLE

include($$ROOT_DIR/src/open/utils/utils.pri)

SOURCES += calibration_job_calibrate.cpp

