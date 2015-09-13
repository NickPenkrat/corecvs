exists(../../../../config.pri) {
    ROOT_DIR=../../../..
    #message(Using global config)
} else { 
    message(Using local config)
    ROOT_DIR=../..
}
ROOT_DIR=$$PWD/$$ROOT_DIR
include($$ROOT_DIR/config.pri)

TEMPLATE = app
TARGET   = test_matcher_basic
CONFIG  += console

!win32: LIBS += -ldl			# load symbol links from "dll/so" files

include(../../utils/utils.pri)

SOURCES += main_feature2d_basic.cpp
