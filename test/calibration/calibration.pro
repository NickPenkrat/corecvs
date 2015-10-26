# try use global config 
exists(../../../../config.pri) {
    ROOT_DIR=../../../..
    #message(Using global config)
    include($$ROOT_DIR/config.pri)
} else { 
    message(Using local config)
    ROOT_DIR=../..
    include($$ROOT_DIR/cvs-config.pri)
}
ROOT_DIR=$$PWD/$$ROOT_DIR

TEMPLATE = app
TARGET   = test_calibration
CONFIG  += console

#DESTDIR = $$ROOT_DIR/bin

include($$ROOT_DIR/src/open/wrappers/gtest/gtest.pri)
include($$ROOT_DIR/src/open/core/core.pri)                        
include($$ROOT_DIR/src/open/utils/utils.pri)                      

SOURCES  += *.cpp
INCLUDES += *.h
