include(../config.pri)
message("OPEN_ROOT_DIRECTORY pro file path : ["$${OPEN_ROOT_DIRECTORY}"]")

include($${OPEN_ROOT_DIRECTORY}/core/core.pri)
include($${OPEN_ROOT_DIRECTORY}/wrappers/gtest/gtest.pri)

#include(affine/affine.pri)

TEMPLATE = app
TARGET   = test_core
CONFIG  += console
CONFIG  -= app_bundle

SOURCES += main.cpp
#SOURCES += affine/main_test_affine.cpp



