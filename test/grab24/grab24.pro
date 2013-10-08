# try use global config
exists(../../../../config.pri) {
    #message(Using global config)
    include(../../../../config.pri)
} else {
    message(Using local config)
    include(../../config.pri)
}


TARGET=test_grab24

TEST_DIR = $$PWD
#TEST_DIR = .
message (Original PWD $$PWD  $$TEST_DIR)
UTILSDIR = $$TEST_DIR/../../utils
include($$UTILSDIR/utils.pri)



SOURCES += main_grab24.cpp

HEADERS += main_grab24.h
