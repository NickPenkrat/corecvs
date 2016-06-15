#ifndef STATUS_TRACKER
#define STATUS_TRACKER

#include <string>
#include <ostream>

#ifdef WITH_TBB
#include <tbb/tbb.h>
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
    bool        isCompleted;
    bool        isFailed;
    bool        isCanceled;

    Status() : currentAction("NONE")
        , completedActions(0), totalActions(0), startedActions(0)
        , completedGlobalActions(0), totalGlobalActions(0), startedGlobalActions(0)
        , isCompleted(false), isFailed(false), isCanceled(false)
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
    ~AutoTracker();

    StatusTracker* st;
};

class StatusTracker
{
public:
    void    setTotalActions(size_t totalActions);
    void    reset(const std::string &action, size_t totalActions);

    void    incrementStarted();
    void    incrementCompleted();
    AutoTracker createAutoTrackerCalculationObject();

    void    setCompleted();
    void    setFailed();
    void    setCanceled();

    bool    isCompleted() const;
    ///
    /// \brief isCanceled
    /// \return Returns whether the setCanceled is called
    ///
    bool    isFailed() const;
    bool    isCanceled() const;

    void    checkCanceled() const;

    bool    isActionCompleted(const std::string &action) const;

    Status  getStatus() const;

private:
    Status  currentStatus;

#ifdef WITH_TBB
    tbb::reader_writer_lock lock;
#endif
    void readLock() const;
    void writeLock();
    void unlock() const;
};

} // namespace corecvs

#endif // STATUS_TRACKER
