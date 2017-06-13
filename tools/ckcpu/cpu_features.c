#include <stdio.h>
#include <intrin.h>

enum {
      MMX = 0
    , SSE   
    , SSE2  
    , SSE3  
    , SSSE3 
    , SSE4_1
    , SSE4_2
    , POPCNT
    , AVX   
    , FMA   
    , AVX2  
};
static const char* features[] = {
      " mmx"
    , " sse"
    , " sse2"
    , " sse3"
    , " ssse3"
    , " sse4.1"
    , " sse4.2"
    , " popcnt"
    , " avx"
    , " fma"
    , " avx2"
};

void main(int argc, char* argv[])
{
    printf("Processor features:");

    // assume any modern CPU has these:
    printf(features[MMX]); printf(features[SSE]); printf(features[SSE2]);

    int info[4];
    __cpuid(info, 1);
    unsigned int reg = info[2];
    if (info[2] & (1u))          printf(features[SSE3]);
    if (info[2] & (1u << 9))     printf(features[SSSE3]);
    if (info[2] & (1u << 19))    printf(features[SSE4_1]);
    if (info[2] & (1u << 20))    printf(features[SSE4_2]);
    if (info[2] & (1u << 23))    printf(features[POPCNT]);
    if (info[2] & (1u << 28))    printf(features[AVX]);
    if (info[2] & (1u << 12))    printf(features[FMA]);
    if (info[1] & (1u << 5))     printf(features[AVX2]);

    printf("\n");
}
