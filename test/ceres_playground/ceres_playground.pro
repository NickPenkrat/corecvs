# try use global config
exists(../../../../config.pri) {
    ROOT_DIR=../../../..
    #message(Using global config)
    include($$ROOT_DIR/config.pri)
} else { 
    #message(Using local config)
    ROOT_DIR=../..
    include($$ROOT_DIR/cvs-config.pri)
}
ROOT_DIR=$$PWD/$$ROOT_DIR

TEMPLATE = app
TARGET   = test_ceres_playground
QT 	+= xml


include(../../utils/utils.pri)                      # it uses TARGET, ROOT_DIR and detects UTILS_BINDIR, OBJECTS_DIR, DESTDIR, ...!


INCLUDEPATH += /usr/include/eigen3
INCLUDEPATH += /usr/include/ceres

SOURCES += main_ceres_playground.cpp \

LIBS += -lceres
LIBS += -lglog
LIBS += -fopenmp
LIBS += -lcsparse
LIBS += -lcxsparse
LIBS += -lcholmod
LIBS += -lopenblas
