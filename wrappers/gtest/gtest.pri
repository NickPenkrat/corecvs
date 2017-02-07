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
		CONFIG(debug, debug|release) {
		    LIBS += -L$$GTEST_PATH/build_x86/Debug   -lgtest
		   #LIBS += -L$$GTEST_PATH/build_x86/Release -lgtest
		}
		CONFIG(release, debug|release) {
		    LIBS += -L$$GTEST_PATH/build_x86/Release -lgtest
		}
    } else {
		CONFIG(debug, debug|release) {
		    LIBS += -L$$GTEST_PATH/build/Debug   -lgtest
		   #LIBS += -L$$GTEST_PATH/build/Release -lgtest
		}
		CONFIG(release, debug|release) {
		    LIBS += -L$$GTEST_PATH/build/Release -lgtest
		}
    }

    !build_pass:message(Using <$$GTEST_PATH/build/Release|Debug>)
}
