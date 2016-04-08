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

TARGET = qtScriptConsole
TEMPLATE = app

include($$ROOT_DIR/src/open/utils/utils.pri)                        # it uses TARGET, ROOT_DIR and detects UTILS_BINDIR, OBJECTS_DIR, DESTDIR, ...!

DESTDIR  = $$ROOT_DIR/bin


SOURCES += main.cpp\
        mainQScriptWindow.cpp

HEADERS  += mainQScriptWindow.h

FORMS    += mainQScriptWindow.ui
