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

TEMPLATE = app
TARGET   = test_genvectorui

include(../../utils/utils.pri)


SOURCES += main_genvectorui.cpp
HEADERS += main_genvectorui.h
