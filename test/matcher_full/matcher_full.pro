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
TARGET=test_matcher_full

TEST_DIR = $$PWD
UTILSDIR = $$TEST_DIR/../../utils
include($$UTILSDIR/utils.pri)

SOURCES += main_matcher_full.cpp

CONFIG += c++11 console

!win32:QMAKE_LFLAGS += -ldl
