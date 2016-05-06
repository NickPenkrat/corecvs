#include <iostream>

#include "statusTracker.h"

void corecvs::StatusTracker::readLock() const
{
#ifdef WITH_TBB
    const_cast<tbb::reader_writer_lock&>(lock).lock_read();
#endif
}
void corecvs::StatusTracker::writeLock()
{
#ifdef WITH_TBB
    lock.lock();
#endif
}
void corecvs::StatusTracker::unlock() const
{
#ifdef WITH_TBB
    const_cast<tbb::reader_writer_lock&>(lock).unlock();
#endif
}

void corecvs::StatusTracker::incrementStarted()
{
    if (this == nullptr)
        return;
    writeLock();
        currentStatus.startedActions++;
        std::cout << "Started: " << currentStatus.startedActions << std::endl;
        CORE_ASSERT_TRUE_S(currentStatus.startedActions <= currentStatus.totalActions);
    unlock();
}

void corecvs::StatusTracker::incrementCompleted()
{
    if (this == nullptr)
        return;
    writeLock();
        currentStatus.completedActions++;
        CORE_ASSERT_TRUE_S(currentStatus.completedActions <= currentStatus.totalActions);
        CORE_ASSERT_TRUE_S(currentStatus.completedActions <= currentStatus.startedActions);
    unlock();
}

void corecvs::StatusTracker::setTotalActions(size_t totalActions)
{
    if (this == nullptr)
        return;
    writeLock();
        currentStatus.completedGlobalActions = currentStatus.startedActions = 0;
        currentStatus.totalGlobalActions = totalActions;
        std::cout << "Total actions: " << currentStatus.totalGlobalActions << std::endl;
    unlock();
}

void corecvs::StatusTracker::reset(const std::string &action, size_t totalActions)
{
    if (this == nullptr)
        return;
    writeLock();
        if(currentStatus.startedGlobalActions > 0)
            currentStatus.completedGlobalActions++;
        currentStatus.startedGlobalActions++;

        currentStatus.currentAction = action;
        currentStatus.completedActions = currentStatus.startedActions = 0;
        currentStatus.totalActions = totalActions;
        std::cout << "Action: " << currentStatus.currentAction
                  << " started: " << currentStatus.startedActions
                  << ", completed " << currentStatus.completedActions
                  << ", total " << currentStatus.totalActions << std::endl;
    unlock();
}

void corecvs::StatusTracker::isActionCompleted(const std::string &action) const
{
    if (this == nullptr)
        return;
    writeLock();
        auto flag = (action == currentStatus.currentAction && currentStatus.totalActions == currentStatus.completedActions);
    unlock();
    return flag;
}

void corecvs::StatusTracker::isCompleted() const
{
    if (this == nullptr)
        return;
    writeLock();
        auto flag = currentStatus.isCompleted;
    unlock();
    return flag;
}

void corecvs::StatusTracker::isFailed() const
{
    if (this == nullptr)
        return;
    writeLock();
        auto flag = currentStatus.isFailed;
    unlock();
    return flag;
}

void corecvs::StatusTracker::setCompleted()
{
    if (this == nullptr)
        return;
    writeLock();
        currentStatus.isCompleted = true;
        std::cout << "Complited!!!" << std::endl;
    unlock();
}

void corecvs::StatusTracker::setFailed()
{
    if (this == nullptr)
        return;
    writeLock();
        currentStatus.isFailed = true;
        std::cout << "Failed!!!" << std::endl;
    unlock();
}

corecvs::Status corecvs::StatusTracker::getStatus() const
{
    if (this == nullptr)
        return corecvs::Status();

    readLock();
        auto status = currentStatus;
    unlock();
    return status;
}
