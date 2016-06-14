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
    if (this == nullptr)
        return;
    writeLock();
        currentStatus.startedActions++;
        std::cout << "Started: " << currentStatus.startedActions << std::endl;
        CORE_ASSERT_TRUE_S(currentStatus.startedActions <= currentStatus.totalActions);
    unlock();
}

void StatusTracker::incrementCompleted()
{
    if (this == nullptr)
        return;

    readLock();
        bool toStop    = currentStatus.stopThread;
        bool hasThrown = currentStatus.hasThrown;
    unlock();

    if (toStop) {
        if (hasThrown) {
            std::cout << "StatusTracker::incrementCompleted ignores throw exception..." << std::endl;
            return;
        }
        writeLock();
            currentStatus.hasThrown = true;
        unlock();
        std::cout << "StatusTracker::incrementCompleted throws exception..." << std::endl;
        throw CancelExecutionException("stopThread!");
    }

    writeLock();
        currentStatus.completedActions++;
        std::cout << "StatusTracker::incrementCompleted " << currentStatus << std::endl;

        CORE_ASSERT_TRUE_S(currentStatus.completedActions <= currentStatus.totalActions);
        CORE_ASSERT_TRUE_S(currentStatus.completedActions <= currentStatus.startedActions);
    unlock();
}

void corecvs::StatusTracker::setTotalActions(size_t totalActions)
{
    if (this == nullptr)
        return;
    writeLock();
        currentStatus.completedGlobalActions = 0;
        currentStatus.totalGlobalActions = totalActions;
        std::cout << "StatusTracker::setTotalActions " << currentStatus.totalGlobalActions << std::endl;
    unlock();
}

void corecvs::StatusTracker::reset(const std::string &action, size_t totalActions)
{
    if (this == nullptr)
        return;
    writeLock();
        if (currentStatus.startedGlobalActions > 0) {
            currentStatus.completedGlobalActions++;
        }
        currentStatus.startedGlobalActions++;

        currentStatus.currentAction    = action;
        currentStatus.completedActions = currentStatus.startedActions = 0;
        currentStatus.totalActions     = totalActions;

        std::cout << "StatusTracker::reset " << currentStatus << std::endl;
    unlock();
}

bool corecvs::StatusTracker::isActionCompleted(const std::string &action) const
{
    if (this == nullptr)
        return false;
    readLock();
        bool flag = (action == currentStatus.currentAction && currentStatus.totalActions == currentStatus.completedActions);
    unlock();
    return flag;
}

bool corecvs::StatusTracker::isCompleted() const
{
    if (this == nullptr)
        return false;
    readLock();
        bool flag = currentStatus.isCompleted;
    unlock();
    return flag;
}

bool corecvs::StatusTracker::isFailed() const
{
    if (this == nullptr)
        return false;
    readLock();
        bool flag = currentStatus.isFailed;
    unlock();
    return flag;
}

bool corecvs::StatusTracker::isCanceled() const
{
    if (this == nullptr)
        return false;
    readLock();
        bool flag = currentStatus.stopThread;
    unlock();
    return flag;
}

void corecvs::StatusTracker::setCompleted()
{
    if (this == nullptr)
        return;
    writeLock();
        currentStatus.isCompleted = true;
        std::cout << "StatusTracker::setCompleted" << std::endl;
    unlock();
}

void corecvs::StatusTracker::setFailed()
{
    if (this == nullptr)
        return;
    writeLock();
        currentStatus.isFailed = true;
        std::cout << "StatusTracker::setFailed" << std::endl;
    unlock();
}

AutoTracker StatusTracker::createAutoTrackerCalculationObject()
{
    if (this == nullptr)
        return 0;
    return AutoTracker(this);
}

void corecvs::StatusTracker::setStopThread()
{
    if (this == nullptr)
        return;
    writeLock();
        currentStatus.stopThread = true;
        std::cout << "StatusTracker::setStopThread" << std::endl;
    unlock();
}

corecvs::Status corecvs::StatusTracker::getStatus() const
{
    if (this == nullptr)
        return Status();

    readLock();
        auto status = currentStatus;
    unlock();
    return status;
}

} // namespace corecvs 
