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
    CONFIG +=    \
                 \
#       debug    \
#        release \

}

buid_pass: message(CONFIG = $$CONFIG)

contains(QMAKE_HOST.arch, "armv7l") {
    message(Odroid platform with ARM detected)
    CONFIG += odroid
} else {
    #message(standard x86/64 platform detected)
}

CONFIG +=           \
                    \
#   trace           \
   asserts          \
                    \
   with_tbb         \
   with_openblas    \   # activate all three libraries,
   with_mkl         \   #  clarification of what to use
   with_fftw        \   #  is done later!
                    \
   with_unorthodox  \   # allow use an experimental filesystem
#   with_ceres       \
#   with_boost       \
#   with_fastbuild  \   # disable optimization for some src on linux
#   with_qscript     \   # experimental...
   \
   with_cusparse    \   # enable CUDA SDK usage for sparse matrix operations by default
   \
   with_opencv      \
   with_nopkgconfig \	# don't automatically detect installed opencv modules


!odroid {               # set the default cpu-features, which to use to build apps
   CONFIG +=       \
      with_sse     \    # all present CPUs support SSE* instructions
      with_sse3    \
      with_sse4_1  \    # all the CPUs Core2 and above support the set of instructions above
#      with_sse4_2 \    # implemented in the Nehalem-based Intel Core i7 product line CPUs; "popcnt" instruction beginning with Nehalem
#      with_avx    \
#      with_avx2   \
#      with_fma    \

   CONFIG += with_native    # it adds some extra features depends on the CPU
}

# set CPU features automatically if "with_native" is active
#
include(config-cpu-features.pri)

disable_cusparse: with_cusparse {
    CONFIG -= with_cusparse
    !build_pass: message (The requested target platform to support disable_cusparse)
}

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

win32 : !set_cpu_sse4_features {
    CONFIG += with_directshow       # by now with activated directshow it crashes on a TabletPC
}

win32 {
    #CONFIG += with_synccam
}

# It's moved to global options above
win32 {
#   CONFIG += with_siftgpu      # activate SiftGPU wrapper that enables using of siftgpu.dll

   #CONFIG += with_hardware     # win32: is disabled for a while
   #CONFIG += with_openglext	# is disabled as it's not yet implemented
   #CONFIG += with_opencl       # delivered OpenCL.lib is compatible with msvc build tools

    CONFIG += with_jsonmodern   # we support only one 3-rd party json reader

    equals(QMAKE_TARGET.arch, "x86") {
        CONFIG -= with_mkl
        CONFIG -= with_openblas
        CONFIG -= with_fftw
        CONFIG -= with_cusparse
        CONFIG -= with_siftgpu
        CONFIG -= with_native
    }

} else:!odroid {
  #CONFIG += with_avcodec
   CONFIG += with_libjpeg
   CONFIG += with_libpng
#  CONFIG += with_rapidjson
   CONFIG += with_jsonmodern
   CONFIG += with_openglext
# TODO: doesn't work for a while with CVS features...  CONFIG += with_siftgpu      # activate SiftGPU wrapper that enables using of siftgpu.so

  #CONFIG += with_hardware     # enabled only for some linux PCs for debugging
  #CONFIG += with_opencl       # Linux: opened for analysis on different GPUs (only nVidia) and CPUs

} else {
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
#    CONFIG += with_opencl       # delivered OpenCL.lib is compatible with msvc build tools
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

