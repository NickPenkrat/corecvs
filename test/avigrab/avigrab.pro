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
TARGET   = test_avigrab

include(../../utils/utils.pri)

LIBS += -lavutil -lavformat -lavcodec -lz -lavutil -lm

SOURCES += main_avigrab.cpp
HEADERS += main_avigrab.h
