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
TARGET  =distortion_corrector
CONFIG += CONSOLE
QT += core
QT -= gui
QT -= xml
QT -= widgets
QT -= opengl

UTILSDIR = $$ROOT_DIR/src/open/utils
include($$UTILSDIR/utils.pri)

#QT -= gui   # is not allowed as this app uses qtFileLoader, which uses QImage that belongs QtGui

SOURCES += distortion_corrector.cpp
