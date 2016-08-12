#ifndef STATUS_TRACKER
#define STATUS_TRACKER

#include <string>
#include <ostream>

#ifdef WITH_TBB
#include <tbb/reader_writer_lock.h>
#endif

#include "global.h"

struct CancelExecutionException : public AssertException
{
    CancelExecutionException(const char* codeExpr) : AssertException(codeExpr) {}
};

namespace corecvs {

struct Status
{
    std::string currentAction;
    size_t      completedActions, totalActions, startedActions, completedGlobalActions, totalGlobalActions, startedGlobalActions;
    bool        isCompleted,
                isFailed,
                isToCancel,
                isCanceled;

    Status() : currentAction("NONE")
        , completedActions(0), totalActions(0), startedActions(0)
        , completedGlobalActions(0), totalGlobalActions(0), startedGlobalActions(0)
        , isCompleted(false), isFailed(false)
        , isToCancel(false), isCanceled(false)
    {}

    friend std::ostream& operator<<(std::ostream& os, const Status &status)
    {
        os << "\tglobal: " << status.startedGlobalActions << "/" << status.completedGlobalActions << "/" << status.totalGlobalActions
            << "\tlocal: " << status.currentAction << " " << status.startedActions << "/" << status.completedActions << "/" << status.totalActions;
        return os;
    }

    std::ostream& progress(std::ostream& os)
    {
        os << *this << std::endl;
        return os;
    }
};

class StatusTracker;

struct AutoTracker
{
    AutoTracker(StatusTracker* st);
    AutoTracker(AutoTracker &&other);
    AutoTracker& operator=(AutoTracker &&other);
    ~AutoTracker();
private:
    AutoTracker(const AutoTracker &other) = delete;
    AutoTracker& operator=(const AutoTracker &other) = delete;
    StatusTracker* st;
};

class StatusTracker
{
public:
    static void SetTotalActions(StatusTracker *tracker, size_t totalActions);
    static void Reset(StatusTracker *tracker, const std::string &action, size_t totalActions);

    static void IncrementStarted(StatusTracker *tracker);
    static void IncrementCompleted(StatusTracker *tracker);
    static AutoTracker CreateAutoTrackerCalculationObject(StatusTracker *tracker);

    static void    SetCompleted(StatusTracker *tracker);
    static void    SetFailed(StatusTracker *tracker);
    static void    SetToCancel(StatusTracker *tracker);
    static void    SetCanceled(StatusTracker *tracker);

    static bool    IsCompleted(const StatusTracker *tracker);
    static bool    IsFailed(const StatusTracker *tracker);

    static bool    IsToCancel(const StatusTracker *tracker);

    static bool    IsCanceled(const StatusTracker *tracker);
    static void    CheckToCancel(const StatusTracker *tracker);

    static bool    IsActionCompleted(const StatusTracker *tracker, const std::string &action);

    static Status  GetStatus(const StatusTracker *tracker);

protected:
    void    setTotalActions(size_t totalActions);
    void    reset(const std::string &action, size_t totalActions);

    void    incrementStarted();
    void    incrementCompleted();
    AutoTracker createAutoTrackerCalculationObject();

    void    setCompleted();
    void    setFailed();
    void    setToCancel();
    void    setCanceled();

    bool    isCompleted() const;
    bool    isFailed() const;
    ///
    /// \brief isToCancel
    /// \return Returns whether the processing task should be canceled asap
    ///
    bool    isToCancel() const;
    ///
    /// \brief isCanceled
    /// \return Returns whether the processing task was canceled and has been stopped
    ///
    bool    isCanceled() const;
    void    checkToCancel() const;

    bool    isActionCompleted(const std::string &action) const;

    Status  getStatus() const;

private:
    Status  currentStatus;

#ifdef WITH_TBB
    typedef tbb::reader_writer_lock::scoped_lock_read read_lock;
    typedef tbb::reader_writer_lock::scoped_lock write_lock;
#else
    typedef int read_lock;
    typedef int write_lock;
#endif

#ifdef WITH_TBB
    mutable tbb::reader_writer_lock
#else
    int
#endif
    lock;
};

} // namespace corecvs

#endif // STATUS_TRACKER
