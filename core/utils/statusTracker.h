#ifndef STATUS_TRACKER
#define STATUS_TRACKER

#include <string>

#ifdef WITH_TBB
#include <tbb/tbb.h>
#endif

#include "global.h"

namespace corecvs {

struct Status
{
    std::string currentAction;
    size_t      completedActions, totalActions, startedActions, completedGlobalActions, totalGlobalActions, startedGlobalActions;
    bool        isCompleted;
    bool        isFailed;

    Status() : currentAction("NONE"), completedActions(0), totalActions(0), startedActions(0), completedGlobalActions(0), totalGlobalActions(0), startedGlobalActions(0)
    {}

};

class StatusTracker
{
public:
    void    incrementStarted();
    void    incrementCompleted();
    void    setTotalActions(size_t totalActions);
    void    reset(const std::string &action, size_t totalActions);
    void    setCompleted();
    void    setFailed();

    bool    isActionCompleted(const std::string &action) const;

    bool    isCompleted() const;
    bool    isFailed() const;

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
