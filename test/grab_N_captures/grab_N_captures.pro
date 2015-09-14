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
TARGET   = test_grab_N_captures
CONFIG  += CONSOLE
QT += core
QT -= gui

include(../../utils/utils.pri)

SOURCES += main_grab_N_captures.cpp
HEADERS += main_grab_N_captures.h
