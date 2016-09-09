/**
 * \file preciseTimer.cpp
 * \brief Add Comment Here
 *
 * \ingroup cppcorefiles
 *

 *
 **/

#if __cplusplus > 199711L
#define CHRONO_CLOCK
#endif

#ifdef WIN32
#define WINDOWS_CLOCK
#endif

#ifdef QT_CLOCK
#include <QTime>
#endif

#ifdef WINDOWS_CLOCK
#include <windows.h>
#elif defined (CHRONO_CLOCK)
#include <chrono>
#else
#include <sys/time.h>
#endif

#include <stdlib.h>

#include "preciseTimer.h"

namespace corecvs {

#ifdef WINDOWS_CLOCK
int64_t PreciseTimer::mFreq = 0L;
#endif

#ifdef CHRONO_CLOCK
using namespace  std::chrono;
#endif

PreciseTimer PreciseTimer::CurrentETime()
{
    PreciseTimer T;

#if defined (CHRONO_CLOCK)
    high_resolution_clock::time_point now = high_resolution_clock::now();
    T.setTime(duration_cast<nanoseconds>(now.time_since_epoch()).count());
#elif defined(QT_CLOCK)
    QTime time = QTime::currentTime();
    T.setTime(((((int64_t)time.hour() * 60 + time.minute()) * 60 + time.second()) * 1000 + time.msec()) * 1000 * 1000);
#elif defined(WINDOWS_CLOCK)
    if (!mFreq) {
        QueryPerformanceFrequency((LARGE_INTEGER*)&mFreq);
    }
    LARGE_INTEGER time;
    QueryPerformanceCounter(&time);
    T.setTime((time.QuadPart * 1000 * 1000 * 1000) / mFreq);
#else
    struct timeval timeVal;
    gettimeofday(&timeVal, NULL);
    T.setTime((timeVal.tv_sec * 1000000 + timeVal.tv_usec) * 1000);
#endif

    return T;
}

} //namespace corecvs
