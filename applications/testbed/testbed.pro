# try use global config 
exists(../../../../config.pri) {
    #message(Using global config)
    include(../../../../config.pri)
} else { 
    message(Using local config)
    include(../../config.pri)
}


TARGET   = testbed
TEMPLATE = app

# Using only open part of the project
UTILSDIR = ../../utils
include($$UTILSDIR/utils.pri)

HEADERS = testbedMainWindow.h \

SOURCES = \
    testbedMainWindow.cpp \
    main_testbed.cpp

FORMS = ui/testbedMainWindow.ui \

RESOURCES += ../../resources/main.qrc
