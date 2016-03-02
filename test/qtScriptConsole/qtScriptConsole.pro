# try use global config
exists(../../../../config.pri) {
    ROOT_DIR=../../../..
    #message(Using global config)
    include($$ROOT_DIR/config.pri)
} else {
    message(Using local config)
    ROOT_DIR=../..
    include($$ROOT_DIR/cvs-config.pri)
}
ROOT_DIR=$$PWD/$$ROOT_DIR

QT       += core gui
QT       += script

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = qtScriptConsole
TEMPLATE = app


SOURCES += main.cpp\
        mainQScriptWindow.cpp

HEADERS  += mainQScriptWindow.h

FORMS    += mainQScriptWindow.ui
