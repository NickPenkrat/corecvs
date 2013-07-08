##################################################################
# matching.pro created on Feb 25, 2011
# This is a file for QMAKE that allows to build the test matching
#
##################################################################
include(../testsCommon.pri)

TARGET = cppunit

SOURCES += \
        cppunit_test.cpp \
        MatcherTest.cpp

HEADERS += MatcherTest.h

LIBS += -L/usr/lib/ \
        -lcppunit

