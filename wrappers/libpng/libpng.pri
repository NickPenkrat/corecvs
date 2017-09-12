isEmpty(LIBPNG_WRAPPER_DIR) {
    message(Incorrect usage of libpng.pri with empty LIBPNG_WRAPPER_DIR. Libpng is switched off!)
} else {
    # include(libpngLibs.pri)
}

with_libpng {
    LIBPNG_PATH = $$(LIBPNG_PATH)
    win32 {
        !isEmpty(LIBPNG_PATH) {
            exists($$LIBPNG_PATH/include/) {
            }
        }
    } else {
        isEmpty(LIBPNG_PATH) {
            !build_pass:message(Compiling with system Libpng)
            exists(/usr/include/libpng/png.h) {
                DEFINES += WITH_LIBPNG
            } else {
                !build_pass:message(libpng not found.)
            }

        } else {
            !build_pass:message(Compiling with libPng from $$LIBPNG_PATH)
            DEFINES += WITH_LIBPNG
        }
    }
}


contains(DEFINES, WITH_LIBPNG) {                    # if it's installed properly with found path for lib

    INCLUDEPATH += $$LIBPNG_WRAPPER_DIR

    HEADERS += \
                $$LIBPNG_WRAPPER_DIR/libpngFileReader.h \

		
    SOURCES += \
                $$LIBPNG_WRAPPER_DIR/libpngFileReader.cpp \

}

