include(../../../config.pri)

TARGET   = baseApplicationStub
TEMPLATE = app

HOSTBASE_DIR=.
include ($$HOSTBASE_DIR/baseApplication.pri)                   # it uses HOSTBASE_DIR, detects HOSTBASE_BINDIR, OBJECTS_DIR, ...

SOURCES += main_vibase.cpp
