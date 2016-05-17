#include <iostream>

#include "statusTracker.h"


namespace corecvs {

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
    writeLock();
        currentStatus.completedActions++;
        CORE_ASSERT_TRUE_S(currentStatus.completedActions <= currentStatus.totalActions);
        CORE_ASSERT_TRUE_S(currentStatus.completedActions <= currentStatus.startedActions);
    unlock();
}

void StatusTracker::reset(const std::string & action, size_t totalActions)
{
    if (this == nullptr)
        return;
    writeLock();
        currentStatus.currentAction = action;
        currentStatus.completedActions = currentStatus.startedActions = 0;
        currentStatus.totalActions = totalActions;
        std::cout << "Action: " << currentStatus.currentAction
                  << " started: " << currentStatus.startedActions
                  << ", completed " << currentStatus.completedActions
                  << ", total " << currentStatus.totalActions << std::endl;
    unlock();
}

Status  StatusTracker::getStatus() const
{
    if (this == nullptr)
        return Status();

    readLock();
        auto status = currentStatus;
    unlock();
    return status;
}

} // namespace corecvs 
