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

TEMPLATE=app
TARGET  = test_new_board_detector
CONFIG += CONSOLE
QT += core
QT -= gui
QT -= xml

include(../../utils/utils.pri)                      # it uses TARGET, ROOT_DIR and detects UTILS_BINDIR, OBJECTS_DIR, DESTDIR, ...!

SOURCES += new_board_detector.cpp

