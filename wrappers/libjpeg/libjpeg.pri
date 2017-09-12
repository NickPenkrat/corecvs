isEmpty(LIBJPEG_WRAPPER_DIR) {
    message(Incorrect usage of libjpeg.pri with empty LIBJPEG_WRAPPER_DIR. Libjpeg is switched off!)
} else {
 #    include(libjpegLibs.pri)
}

with_libjpeg {
    LIBJPEG_PATH = $$(LIBJPEG_PATH)
    win32 {
        !isEmpty(LIBJPEG_PATH) {
            exists($$LIBJPEG_PATH/include/) {
                INCLUDEPATH += $$LIBJPEG_PATH/include                            # when we made "make install" there
            }
        }
    } else {
        isEmpty(LIBJPEG_PATH) {
            !build_pass:message(Compiling with system Libjpeg)
            exists(/usr/include/jpeglib.h) {
                DEFINES += WITH_LIBJPEG
            } else {
                !build_pass:message(Libjpeg not found.)
            }
        } else {
            !build_pass:message(Compiling with libJpeg from $$LIBJPEG_PATH)
            DEFINES += WITH_LIBJPEG
        }
    }
}

contains(DEFINES, WITH_LIBJPEG) {                    # if it's installed properly with found path for lib

    INCLUDEPATH += $$LIBJPEG_WRAPPER_DIR

    HEADERS += \
                $$LIBJPEG_WRAPPER_DIR/libjpegFileReader.h \

		
    SOURCES += \
                $$LIBJPEG_WRAPPER_DIR/libjpegFileReader.cpp \

}

