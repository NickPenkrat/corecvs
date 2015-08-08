# This file is used internally for each UnitTest
#
# input1 parameter: $$OBJ_TESTS_DIRNAME    - name of common intermediate dir for all UnitTests of the current project
# input2 parameter: $$USE_CORE_PRI_FILE    - required core|rescore project file to include
#
# input3 parameter: $$COREDIR              - possible requiring    core folder path (when "core.pri" is including)
# input4 parameter: $$RES_COREDIR          - possible requiring rescore folder path (when "core_restricted.pri" is including)
#

# try use global config 
exists(../../../config.pri) {
    ROOT_DIR=../../..
    #message(Using global config)
} else { 
    message(Using local config)
    ROOT_DIR=..
}
#!win32 {                                            # it dues to the "mocinclude.tmp" bug on win32!
    ROOT_DIR=$$PWD/$$ROOT_DIR
#}
#!build_pass: message(Tests root dir is $$ROOT_DIR)
include($$ROOT_DIR/config.pri)

CONFIG += console

win32-msvc* {
    # Sometimes mt.exe fails on embedding action in parallel making...
    #CONFIG -= embed_manifest_exe
}

DESTDIR = $$ROOT_DIR/bin
#COREDIR = $$ROOT_DIR/$$COREDIR
#RES_COREDIR = $$ROOT_DIR/$$RES_COREDIR

#message(We Using core $$USE_CORE_PRI_FILE)
#message(  With    coredir $$COREDIR)
#message(  With rescoredir $$RES_COREDIR)
include($$USE_CORE_PRI_FILE)


TARGET_ORIG = $$TARGET                              # store original target name for proper detection of the obj.dir
!contains(OBJ_TESTS_DIRNAME, tests_restricted) {    # first include file is "testsCommon.pri" (open tests), second - "testsRestricted.pri" (restricted tests)
    TARGET = $$join(TARGET,,test_,)                 # use target name common format for all open tests as  "test_<name of the test>"
} else {
    TARGET = $$join(TARGET,,test_res_,)             # use target name common format for all restricted tests as  "test_res_<name of the test>"
}
TARGET = $$join(TARGET,,,$$BUILD_CFG_SFX)           # add 'd' at the end for debug versions

OBJECTS_DIR = $$ROOT_DIR/.obj/$$OBJ_TESTS_DIRNAME/$$TARGET_ORIG$$BUILD_CFG_NAME

MOC_DIR = $$OBJECTS_DIR                             # we have to set it to omit creating dummy dirs: debug,release
