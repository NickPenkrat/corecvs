GTEST_PATH = $$(GTEST_PATH)
win32 {
    isEmpty(GTEST_PATH) {
       message(GTEST_PATH not configured!)
     } else {
       INCLUDEPATH += $(GTEST_PATH)/include
       LIBS += -L$(GTEST_PATH)/build/Debug -lgtest
       LIBS += -L$(GTEST_PATH)/build/Release -lgtest
    }
} else {
    isEmpty(GTEST_PATH) {
        LIBS += -lgtest
        message(Using system default google test)
    } else {
        # have no idea about how somebody will store goolgetest libs with manually built googletest
       INCLUDEPATH += $(GTEST_PATH)/include
       QMAKE_LFLAGS += -L$(GTEST_PATH)/Release -L$(GTEST_PATH)/Debug
       LIBS += -lgtest
    }
}
