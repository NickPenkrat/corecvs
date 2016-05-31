#include "statusTracker.h"
#include <iostream>

namespace corecvs {

AutoTracker::AutoTracker(StatusTracker* st) : st(st)
{
    st->incrementStarted();
}

AutoTracker::~AutoTracker()
{
    st->incrementCompleted();
}

void StatusTracker::readLock() const
{
#ifdef WITH_TBB
    const_cast<tbb::reader_writer_lock&>(lock).lock_read();
#endif
}
void StatusTracker::writeLock()
{
#ifdef WITH_TBB
    lock.lock();
#endif
}
void StatusTracker::unlock() const
{
#ifdef WITH_TBB
    const_cast<tbb::reader_writer_lock&>(lock).unlock();
#endif
}

void StatusTracker::incrementStarted()
{
    writeLock();
        currentStatus.startedActions++;
        std::cout << "Started: " << currentStatus.startedActions << std::endl;
        CORE_ASSERT_TRUE_S(currentStatus.startedActions <= currentStatus.totalActions);
    unlock();
}

void StatusTracker::incrementCompleted()
{
    readLock();
        bool toStop = currentStatus.stopThread;
    unlock();

    if (toStop) {
        throw AssertException("stopThread!");
    }

    writeLock();
        currentStatus.completedActions++;
        std::cout << "StatusTracker::incrementCompleted(): " << currentStatus << std::endl;

        CORE_ASSERT_TRUE_S(currentStatus.completedActions <= currentStatus.totalActions);
        CORE_ASSERT_TRUE_S(currentStatus.completedActions <= currentStatus.startedActions);
    unlock();
}

void corecvs::StatusTracker::setTotalActions(size_t totalActions)
{
    writeLock();
        currentStatus.completedGlobalActions = 0;
        currentStatus.totalGlobalActions = totalActions;
        std::cout << "Total actions: " << currentStatus.totalGlobalActions << std::endl;
    unlock();
}

void corecvs::StatusTracker::reset(const std::string &action, size_t totalActions)
{
    writeLock();
        if (currentStatus.startedGlobalActions > 0) {
            currentStatus.completedGlobalActions++;
        }
        currentStatus.startedGlobalActions++;

        currentStatus.currentAction    = action;
        currentStatus.completedActions = currentStatus.startedActions = 0;
        currentStatus.totalActions     = totalActions;

        std::cout << "StatusTracker::reset(): " << currentStatus << std::endl;
    unlock();
}

bool corecvs::StatusTracker::isActionCompleted(const std::string &action) const
{
    readLock();
        bool flag = (action == currentStatus.currentAction && currentStatus.totalActions == currentStatus.completedActions);
    unlock();
    return flag;
}

bool corecvs::StatusTracker::isCompleted() const
{
    readLock();
        bool flag = currentStatus.isCompleted;
    unlock();
    return flag;
}

bool corecvs::StatusTracker::isFailed() const
{
    readLock();
        bool flag = currentStatus.isFailed;
    unlock();
    return flag;
}

bool corecvs::StatusTracker::isCanceled() const
{
    readLock();
        bool flag = currentStatus.stopThread;
    unlock();
    return flag;
}

void corecvs::StatusTracker::setCompleted()
{
    writeLock();
        currentStatus.isCompleted = true;
        std::cout << "StatusTracker::setCompleted()" << std::endl;
    unlock();
}

void corecvs::StatusTracker::setFailed()
{
    writeLock();
        currentStatus.isFailed = true;
        std::cout << "StatusTracker::setFailed()" << std::endl;
    unlock();
}

AutoTracker StatusTracker::createAutoTrackerCalculationObject()
{
    return AutoTracker(this);
}

void corecvs::StatusTracker::setStopThread()
{
    writeLock();
        currentStatus.stopThread = true;
        std::cout << "setStopThread() is called." << std::endl;
    unlock();
}

corecvs::Status corecvs::StatusTracker::getStatus() const
{
    readLock();
        auto status = currentStatus;
    unlock();
    return status;
}

} // namespace corecvs 
