include(../../../config.pri)

include($${OPEN_ROOT_DIRECTORY}/core/core.pri)
include($${OPEN_ROOT_DIRECTORY}/wrappers/gtest/gtest.pri)

TEMPLATE = app
TARGET   = test_core
CONFIG  += console
CONFIG  -= app_bundle

SOURCES += main.cpp \
affine/main_test_affine.cpp \
arithmetics/main_test_arithmetics.cpp \
assignment/main_test_assignment.cpp \
automotive/main_test_automotive.cpp \
buffer/main_test_buffer.cpp




