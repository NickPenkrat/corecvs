with_jsonmodern {

    !c++11 {
        message(Json for Modern C++ is switched off! It requires c++11 standard)
    } else : win32-msvc2010 {
        message(Json for Modern C++ is switched off! It does not support MSVC<2015)
    } else : win32-msvc2013 {
        message(Json for Modern C++ is switched off! It does not support MSVC<2015)
    } else {
        message(Json for Modern C++ module is ready for use)

        JSONMODERN_PATH = $$(JSONMODERN_PATH)
        DEFINES += WITH_JSONMODERN
    }
}
