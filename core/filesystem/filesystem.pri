HEADERS += \
    filesystem/folderScanner.h 
   
SOURCES += \
    filesystem/folderScanner.cpp

!win32 {
    LIBS += -lstdc++fs
}
