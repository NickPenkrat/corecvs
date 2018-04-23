TEMPLATE = subdirs
CONFIG  += ordered

SUBDIRS +=                   \
##    generator              \   	# It needs core. For true rebuild after it's built, it should regenerate sources and then we should build projects once more.
    core                     \
    utils                    \
    test-core                \
#    test-core-perf           \
    tests_big                \
    \   


#utility applications
CONFIG += with_minitools
with_minitools {

  message (Minitools On)

  SUBDIRS +=                 \
    cloudview                \
#    rectifier                \
    imageview                \
    \
#    imageAugment             \
#    testbed                  \
#    base_application         \
#    base_application_example \
#    recorder                 \
#    merger                   \

}


win32 : !set_cpu_sse4_features {
    SUBDIRS += directshow
    directshow.file       = wrappers/directShow/directShow.pro
    directshow.depends   += core
}

generator.depends                += core
utils.depends                    += core
test-core.depends                += core
test-core-perf.depends           += core

tests_big.depends                += core utils

testbed.depends                  += core utils
base_application.depends         += core utils
base_application_example.depends += base_application
recorder.depends                 += base_application
merger.depends                   += base_application

cloudview.depends                += utils
rectifier.depends                += utils
imageview.depends                += utils
imageAugment.depends             += utils

core.file                         = core/core.pro
utils.file                        = utils/utils.pro
test-core.file                    = test-core/test-core.pro
test-core-perf.file               = test-core/-perf/test-core-perf.pro
tests_big.file                    = test/tests.pro
base_application.file             = applications/base/baseApplication.pro
base_application_example.file     = applications/base/baseApplicationExample.pro
recorder.file                     = applications/recorder/recorder.pro
merger.file                       = applications/merger/merger.pro
testbed.file                      = applications/testbed/testbed.pro
cloudview.file                    = applications/cloudview/cloudview.pro
imageview.file                    = applications/imageview/imageview.pro
rectifier.file                    = applications/rectifier/rectifier.pro
imageAugment.file                 = applications/imageAugment/imageAugment.pro

generator.file                    = tools/generator/generator.pro
vcprojFix.file                    = tools/vcprojFix/vcprojFix.pro

