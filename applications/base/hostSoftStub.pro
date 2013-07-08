include(../../../config.pri)

TARGET   = vihostbase
TEMPLATE = app

HOSTBASE_DIR=.
include ($$HOSTBASE_DIR/hostBase.pri)                   # it uses HOSTBASE_DIR, detects HOSTBASE_BINDIR, OBJECTS_DIR, ...

SOURCES += main_vibase.cpp
