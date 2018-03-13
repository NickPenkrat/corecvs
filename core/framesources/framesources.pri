SOURCES += \
    $$PWD/*.cpp \
    $$PWD/file/*.cpp \

HEADERS += \
    $$PWD/*.h \
    $$PWD/file/*.h \


# PREC
CONFIG += with_framesource_prec

with_framesource_prec {
#HEADERS +=  framesources/file/precCapture.h
#SOURCES +=  framesources/file/precCapture.cpp
DEFINES += WITH_FRAMESOURCE_PREC
}

# FILE
CONFIG += with_framesource_file

with_framesource_file {
#HEADERS +=  framesources/file/fileCapture.h
#SOURCES +=  framesources/file/fileCapture.cpp
DEFINES += WITH_FRAMESOURCE_FILE
}
