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
TARGET   = testbed
QT      += xml

include(../../utils/utils.pri)                      # it uses TARGET, ROOT_DIR and detects UTILS_BINDIR, OBJECTS_DIR, DESTDIR, ...!


HEADERS = \
    testbedMainWindow.h \
#    pointScene.h \


SOURCES = \
    testbedMainWindow.cpp \
    main_testbed.cpp \
#    pointScene.cpp \

FORMS = ui/testbedMainWindow.ui \

#RESOURCES += ../../resources/main.qrc
