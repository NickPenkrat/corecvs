# try use global config
exists(../../config.pri) {
    ROOT_DIR=../..
    #message(Using global config)
} else {
    message(Using local config)
    ROOT_DIR=..
}
ROOT_DIR=$$PWD/$$ROOT_DIR
#!build_pass: message(Tests root dir is $$ROOT_DIR)
include($$ROOT_DIR/config.pri)

include(../core/core.pri)
include(../wrappers/gtest/gtest.pri)

#message(INCLUDEPATH=$$INCLUDEPATH)

TEMPLATE = app
TARGET   = test_core
CONFIG  += console
CONFIG  -= app_bundle

#INCLUDEPATH += "C:/Tools/gtest-1.7.0/include"
#LIBS += -L"C:/Tools/gtest-1.7.0/build/Debug" -lgtest

SOURCES += main.cpp
