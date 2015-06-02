# try use global config 
exists(../../../../config.pri) {
    ROOT_DIR=../../../..
    #message(Using global config)
} else { 
    message(Using local config)
    ROOT_DIR=../..
}
#!win32 {                                            # it dues to the "mocinclude.tmp" bug on win32!
    ROOT_DIR=$$PWD/$$ROOT_DIR
#}
include($$ROOT_DIR/config.pri)

TEMPLATE=app
TARGET=test_grab_N_captures
CONFIG+=CONSOLE
QT+=core gui xml

TEST_DIR = $$PWD
#TEST_DIR = .
#message (Original PWD $$PWD  $$TEST_DIR)
#COREDIR=../../core
#include($$COREDIR/core.pri)                         # it uses COREDIR, TARGET and detects     COREBINDIR!

UTILSDIR = $$TEST_DIR/../../utils
include($$UTILSDIR/utils.pri)

SOURCES += main_grab_N_captures.cpp

HEADERS += main_grab_N_captures.h

core.file                = src/open/core/core.pro
#unitTests.file           = src/open/test-core/unitTests.pro
utils.file               = src/open/utils/utils.pro
