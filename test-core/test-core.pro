exists(../../config.pri) {
    ROOT_DIR=../..
} else {
    message(Using local config)
    ROOT_DIR=..
}
ROOT_DIR=$$PWD/$$ROOT_DIR

include($$ROOT_DIR/config.pri)
include(../core/core.pri)
include(../wrappers/gtest/gtest.pri)

TEMPLATE = app
TARGET   = test_core
CONFIG  += console
CONFIG  -= app_bundle

SOURCES += main.cpp
