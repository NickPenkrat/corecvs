#ifndef STATUS_TRACKER_CATCHER
#define STATUS_TRACKER_CATCHER

#include <string>
#include <ostream>

#include "core/tbbwrapper/tbbWrapper.h"
#include <functional>
#include "core/utils/global.h"

namespace {

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

#ifndef I_WANT_A_SKY_TO_FALL_ONTO_MY_HEAD
#ifdef WITH_TBB
#define CATCH_BLOCK_FOR_ANY_METHOD_SDK(processState)            \
                                                                \
    catch (const CancelExecutionException &ex)      /* on Windows with TBB 4.3.6 (with working 'exact exception propagation')*/ \
    {                                                           \
        cout << "CancelExecutionException handler:" << StatusTracker::GetStatus(processState) << endl; \
        L_INFO << ex.what();                                    \
        /*CORE_ASSERT_TRUE_S(StatusTracker::IsToCancel(processState));         */\
        /*CORE_ASSERT_FALSE_S(StatusTracker::IsCanceled(processState));        */ \
        StatusTracker::SetCanceled(processState);                            \
    }                                                           \
    catch (const tbb::captured_exception &ex)       /* on Linux with TBB 4.3.6 (without working 'exact exception propagation')*/ \
    {                                                           \
        cout << "tbb::captured_exception handler:" << StatusTracker::GetStatus(processState) << endl; \
        if (!StatusTracker::IsToCancel(processState)) {                      \
            L_ERROR << ex.what();                   /* we've got some unexpected exception wrapped by TBB - it's a really error */ \
            StatusTracker::SetFailed(processState, ex.what());                          \
                }                                                       \
                else {                                                  \
            L_INFO << ex.what();                                \
            /*CORE_ASSERT_FALSE_S(StatusTracker::IsCanceled(processState));    */\
            StatusTracker::SetCanceled(processState);                        \
        }                                                       \
    }                                                           \
    catch (const std::exception &ex)                            \
    {                                                           \
        cout << "std::exception handler:" << StatusTracker::GetStatus(processState) << endl; \
        L_ERROR << ex.what();                       /* we've got some unexpected exception */ \
        StatusTracker::SetFailed(processState, ex.what());      \
    }                                                           \
    catch (...)                                                 \
    {                                                           \
        L_ERROR << "An unhandled exception in SDK!";            \
        StatusTracker::SetFailed(processState, "An unhandled exception in SDK!"); \
    }                                                           \

#else
#define CATCH_BLOCK_FOR_ANY_METHOD_SDK(processState)            \
                                                                \
    catch (const CancelExecutionException &ex)      /* on Windows with TBB 4.3.6 (with working 'exact exception propagation')*/ \
    {                                                           \
        cout << "CancelExecutionException handler:" << StatusTracker::GetStatus(processState) << endl; \
        L_INFO << ex.what();                                    \
        CORE_ASSERT_TRUE_S(StatusTracker::IsToCancel(processState));         \
        CORE_ASSERT_FALSE_S(StatusTracker::IsCanceled(processState));        \
        StatusTracker::SetCanceled(processState);                            \
    }                                                           \
    catch (const std::exception &ex)                            \
    {                                                           \
        cout << "std::exception handler:" << StatusTracker::GetStatus(processState) << endl; \
        L_ERROR << ex.what();                       /* we've got some unexpected exception */ \
        StatusTracker::SetFailed(processState, ex.what());      \
    }                                                           \
    catch (...)                                                 \
    {                                                           \
        L_ERROR << "An unhandled exception in SDK!";            \
        StatusTracker::SetFailed(processState, "An unhandled exception in SDK!"); \
    }                                                           \

#endif
#else
#define CATCH_BLOCK_FOR_ANY_METHOD_SDK(processState)            \
                                                                \
    catch (int)                                                 \
    {                                                           \
        L_ERROR << "Somebody throws ints in SDK, and you've asked a sky to fall onto your head!!!111oneoneone";            \
        StatusTracker::SetFailed(processState, "-1");           \
        exit(-1);                                               \
    }                                                           \

#endif

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

} // namespace corecvs

#endif // STATUS_TRACKER_CATCHER
