isEmpty(JSONMODERN_WRAPPER_DIR) {
    message(Incorrect usage of jsonmodern.pri with empty JSONMODERN_WRAPPER_DIR. Json for Modern C++ is switched off!)
} else {
    !exists($$JSONMODERN_WRAPPER_DIR/sources/CMakeLists.txt) {
        message(Json for Modern C++ is switched off! Not cloned.)
    } else {
        message(Json for Modern C++ found!)

        include(jsonmodernLibs.pri)
    }
}

contains(DEFINES, WITH_JSONMODERN) {                    # if it's installed properly with found path for lib

    INCLUDEPATH += $$JSONMODERN_WRAPPER_DIR
    INCLUDEPATH += $$JSONMODERN_WRAPPER_DIR/sources/src/


    HEADERS +=  $$JSONMODERN_WRAPPER_DIR/jsonModernReader.h
    SOURCES +=  $$JSONMODERN_WRAPPER_DIR/jsonModernReader.cpp


#    HEADERS +=  $$RAPIDJSON_WRAPPER_DIR/rapidJSONWriter.h
#    SOURCES +=  $$RAPIDJSON_WRAPPER_DIR/rapidJSONWriter.cpp

}

