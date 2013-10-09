# try use global config
exists(../../../../config.pri) {
    ROOT_DIR=../../../..
} else { 
    message(Using local config)
    ROOT_DIR=../..
}
include($$ROOT_DIR/config.pri)


TARGET   = testbed
TEMPLATE = app

#OBJECTS_DIR = $$ROOT_DIR/.obj
#MOC_DIR = $$ROOT_DIR/.obj
#UI_DIR  = $$ROOT_DIR/.obj

# Using only open part of the project
UTILSDIR = ../../utils
include($$UTILSDIR/utils.pri)

win32 {
    MOC_DIR = ../../../../.obj/$$TARGET_ORIG/$$BUILD_CFG_NAME  # resolve moc path for mocs to help qmake to unify those paths.
}

HEADERS = testbedMainWindow.h \

SOURCES = \
    testbedMainWindow.cpp \
    main_testbed.cpp

FORMS = ui/testbedMainWindow.ui \

RESOURCES += ../../resources/main.qrc
