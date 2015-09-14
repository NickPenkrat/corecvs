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
TARGET   = distortion_corrector
CONFIG  += CONSOLE
QT += core
QT -= xml
QT -= widgets
QT -= opengl
#QT -= gui   # is not allowed as this app uses qtFileLoader, which uses QImage that belongs QtGui

include($$ROOT_DIR/src/open/utils/utils.pri)

SOURCES += distortion_corrector.cpp
