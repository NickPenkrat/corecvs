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
TARGET   = test_serialize1
QT += xml

include(../../utils/utils.pri)

SOURCES += main_serialize1.cpp
HEADERS += main_serialize1.h
