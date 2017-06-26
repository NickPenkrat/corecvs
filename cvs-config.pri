# Configuration of a build is determined by CONFIG variable, comment and uncomment
# entries as needed. Configurations can be added from command line, like
# qmake "CONFIG+=with_neon" cvs.pro


# This is a common section, anyway everybody is changing config.pri after checkout

# Open both lines to work with only one configuration that is chosen below.
# Such a config would have alone makefile at each working src dir.
#
#CONFIG -= debug_and_release
#CONFIG -= debug_and_release_target

!contains(CORECVS_INCLUDED, "config.pri") {
CORECVS_INCLUDED +=  config.pri

# By generating VS projects only, neither debug nor release must be chosen!
# For makefiles we ask to generate both config makefiles and build only one chosen below config.
#
!gen_vsproj {
    CONFIG +=   \
                \
#        debug \
#        release \

}

CONFIG +=        \
                 \
#   trace        \
   asserts       \
                 \
   with_native   \
   with_tbb      \
#   with_openblas \      # activate all three libraries,
#   with_mkl      \      #  clarify what
#   with_fftw     \      #  to use later!
#   with_opencv   \
#   with_fastbuild \    # disable optimization for some src on linux
#   with_unorthodox \   # allow use an experimental filesystem
   with_cusparse \     # enable CUDA SDK usage for sparse matrix operations

include(config-cpu-features.pri)

!win32:!macx {
    CONFIG +=             \
                          \
    #   pedantic_build    \
    #   gcc_env_toolchain \
    #   gcc48_toolchain   \
    #   gcc_lto           \
    #   gcc_checker       \
    #   gcc47_toolchain   \
    #   gcc45_toolchain   \
    #   clang_toolchain   \
    #   clang_analyser    \
    #   ccache_toolchain  \
    #   icc_toolchain     \
    #   arm_toolchain     \
    #    with_neon        \
    #\
    #    profile_alignment \
    #    profile_prepare   \
    #    profile_use       \

}

CONFIG +=                   \
#        with_ueye           \
        with_httpserver

win32 {
    CONFIG +=               \
        with_directshow     \
#         with_synccam        \
         
}

# It's moved to global options above
win32 {
#   CONFIG += with_siftgpu      # activate SiftGPU wrapper that enables using of siftgpu.dll

#    CONFIG += with_opencv
   #CONFIG += with_hardware     # win32: is disabled for a while
   #CONFIG += with_openglext	# is disabled as it's not yet implemented
   #CONFIG += with_opencl       # delivered OpenCL.lib is compatible with msvc build tools
} else:!odroid {
 #  CONFIG += with_opencv
  #CONFIG += with_avcodec
 #  CONFIG += with_libjpeg
 #  CONFIG += with_libpng
 #  CONFIG += with_openglext
#   CONFIG += with_siftgpu      # activate SiftGPU wrapper that enables using of siftgpu.so

  #CONFIG += with_hardware     # enabled only for some linux PCs for debugging
  #CONFIG += with_opencl       # Linux: opened for analysis on different GPUs (only nVidia) and CPUs

} else {
#   CONFIG += with_opencv       # we need to install it on Odroid
   CONFIG += with_avcodec
}

win32 {
   #CONFIG += with_x11extras    # don't use x11extras library on win
} else:!macx {
  qtHaveModule(x11extras) {
    !build_pass: message(Using found plugin x11extras)
    CONFIG += with_x11extras    # activate usage x11extras library for drawing overlay
  }
} else {
   #CONFIG += with_x11extras    # don't use x11extras library on mac
}

win32-msvc* {
 #   CONFIG += with_opencl       # delivered OpenCL.lib is compatible with msvc build tools
} else:win32 {
                                # it's not supported as delivered OpenCL.lib isn't compatible with the win32-mingw linker tool
#    CONFIG += with_opencl       # opened as it's managed more carefully by the opencl project
} else {
#    CONFIG += with_opencl       # Linux: opened for analysis on different GPUs (only nVidia) and CPUs
}

# include standard part for any project that tunes some specific parameters that depend of the config been set above
#
include(./common.pri)

} # !contains(CORECVS_INCLUDED, "config.pri")

