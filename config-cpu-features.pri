#
# Autodetect CPU features block
#
CPU_FLAGS = ""

with_native {
    !win32 {
        CPU_FLAGS += $$system(cat /proc/cpuinfo | grep -m 1 "^flags")
    } else {
        CPU_FLAGS_PATH=$$shell_path($$PWD/tools/ckcpu/cpu_features.exe)
        !exists("$$CPU_FLAGS_PATH") : CPU_FLAGS_PATH=$$shell_path($$PWD/siblings/ckcpu/cpu_features.exe)
        exists("$$CPU_FLAGS_PATH") {
            CPU_FLAGS += $$system("$$CPU_FLAGS_PATH")
        } else {
            message (CPU_FLAGS_PATH=$$CPU_FLAGS_PATH not found!)
        }
    }
    #message (Platform natively supports $$CPU_FLAGS)
}

contains(CPU_FLAGS, "sse") {
    CONFIG += with_sse
    !build_pass: contains(TARGET, core): message (Natively support SSE)
}
contains(CPU_FLAGS, "sse2") {
    CONFIG += with_sse2
    !build_pass: contains(TARGET, core): message (Natively support SSE2)
}
contains(CPU_FLAGS, "sse3") {
    CONFIG += with_sse3
    !build_pass: contains(TARGET, core): message (Natively support SSE3)
}
contains(CPU_FLAGS, "ssse3") {
    CONFIG += with_sse3
    !build_pass: contains(TARGET, core): message (Natively support SSSE3)
}
contains(CPU_FLAGS, "sse4.1") {
    CONFIG += with_sse4_1
    !build_pass: contains(TARGET, core): message (Natively support SSE4.1)
}
contains(CPU_FLAGS, "sse4.2") {
    CONFIG += with_sse4_2
    !build_pass: contains(TARGET, core): message (Natively support SSE4.2)
}
contains(CPU_FLAGS, "popcnt") {
    !build_pass: contains(TARGET, core): message (Natively support POPCNT)
    !contains(CONFIG, with_sse4_2): message (!!!!! FIXME !!!!!! Natively support POPCNT, but not SSE4.2 !!!!!!!!!!!!)
}
contains(CPU_FLAGS, "avx") {
    CONFIG += with_avx
    !build_pass: contains(TARGET, core): message (Natively support AVX)
}
contains(CPU_FLAGS, "avx2") {
    CONFIG += with_avx2
    !build_pass: contains(TARGET, core): message (Natively support AVX2)
}
contains(CPU_FLAGS, "fma") {
    CONFIG += with_fma
    !build_pass: contains(TARGET, core): message (Natively support FMA)
}
