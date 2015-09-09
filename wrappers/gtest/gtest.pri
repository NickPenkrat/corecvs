GTEST_PATH = $$(GTEST_PATH)
isEmpty(GTEST_PATH) {
    message(GTEST_PATH not configured!)
} else {
    INCLUDEPATH += $(GTEST_PATH)/include
    LIBS += -L$(GTEST_PATH)/build/Debug -lgtest
    LIBS += -L$(GTEST_PATH)/build/Release -lgtest
}
