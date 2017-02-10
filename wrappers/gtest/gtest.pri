GTEST_PATH = $$(GTEST_PATH)
isEmpty(GTEST_PATH) {
    win32 {
        message(GTEST_PATH not configured!)
    } else {
        LIBS += -lgtest
    }
} else {
    INCLUDEPATH += $$GTEST_PATH/include

    equals(QMAKE_TARGET.arch, "x86") {
		GTEST_PATH_BUILD = $$GTEST_PATH/build_x86
    } else {
		GTEST_PATH_BUILD = $$GTEST_PATH/build
    }
    
    CONFIG(debug, debug|release) {
        LIBS += -L$$GTEST_PATH_BUILD/Debug   -lgtest
    }
    CONFIG(release, debug|release) {
        LIBS += -L$$GTEST_PATH_BUILD/Release -lgtest
    }

    !build_pass:message(Using <$$GTEST_PATH/build/Release|Debug>)
}
