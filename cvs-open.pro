TEMPLATE = subdirs
CONFIG  += ordered

SUBDIRS +=                   \
    core                     \
    unitTests                \
    utils                    \
    base_application         \
    base_application_example \    
    recorder                 \
    testbed                  \
    

win32 {
    SUBDIRS += directshow
}

unitTests.depends                += core
utils.depends                    += core
directshow.depends               += core
base_application.depends         += core            # must be utils: as libs building could be done in parallel except apps!
testbed.depends                  += core            # must be utils: as libs building could be done in parallel except apps!
recorder.depends                 += base_application
base_application_example.depends += base_application

core.file                         = core/core.pro
unitTests.file                    = test-core/unitTests.pro
utils.file                        = utils/utils.pro
base_application.file             = applications/base/baseApplication.pro
base_application_example.file     = applications/base/baseApplicationExample.pro
recorder.file                     = applications/recorder/recorder.pro
testbed.file                      = applications/testbed/testbed.pro
directshow.file                   = wrappers/directShow/directShow.pro
