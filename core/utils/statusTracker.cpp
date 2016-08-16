#include "statusTracker.h"
#include "tbbWrapper.h"

#include <iostream>

namespace corecvs {

void        StatusTracker::SetTotalActions(StatusTracker *st, size_t totalActions)                 { if (st) st->setTotalActions(totalActions); }
void        StatusTracker::Reset(StatusTracker *st, const std::string &action, size_t totalActions){ if (st) st->reset(action, totalActions); }
void        StatusTracker::IncrementStarted(StatusTracker *st)                                     { if (st) st->incrementStarted();   }
void        StatusTracker::IncrementCompleted(StatusTracker *st)                                   { if (st) st->incrementCompleted(); }
AutoTracker StatusTracker::CreateAutoTrackerCalculationObject(StatusTracker *st)                   { return st ? st->createAutoTrackerCalculationObject() : AutoTracker(nullptr); }
void        StatusTracker::SetCompleted(StatusTracker *st)                                         { if (st) st->setCompleted(); }
void        StatusTracker::SetFailed(StatusTracker *st)                                            { if (st) st->setFailed();    }
void        StatusTracker::SetToCancel(StatusTracker *st)                                          { if (st) st->setToCancel();  }
void        StatusTracker::SetCanceled(StatusTracker *st)                                          { if (st) st->setCanceled();  }
bool        StatusTracker::IsCompleted(const StatusTracker *st)                                    { return st ? st->isCompleted() : false; }
bool        StatusTracker::IsFailed(const StatusTracker *st)                                       { return st ? st->isFailed()    : false; }
bool        StatusTracker::IsToCancel(const StatusTracker *st)                                     { return st ? st->isToCancel()  : false; }
bool        StatusTracker::IsCanceled(const StatusTracker *st)                                     { return st ? st->isCanceled()  : false; }
void        StatusTracker::CheckToCancel(const StatusTracker *st)                                  { if (st) st->checkToCancel(); }
bool        StatusTracker::IsActionCompleted(const StatusTracker *st, const std::string &action)   { return st ? st->isActionCompleted(action) : false; }
Status      StatusTracker::GetStatus(const StatusTracker *st)                                      { return st ? st->getStatus() : Status(); }

AutoTracker::AutoTracker(StatusTracker* st) : st(st)
{
    StatusTracker::IncrementStarted(st);
}

AutoTracker& AutoTracker::operator=(AutoTracker &&other)
{
    // other is a complete object => started is already incremented,
    // so the only action that is needed is to steal ST pointer from
    // an argument
    std::swap(st, other.st);
    return *this;
}

AutoTracker::AutoTracker(AutoTracker&& other) : st(other.st)
{
    // other is a complete object => started is already incremented,
    // so the only action that is needed is to steal ST pointer from
    // an argument
    other.st = nullptr;
}

AutoTracker::~AutoTracker()
{
    StatusTracker::IncrementCompleted(st);
}

void corecvs::StatusTracker::setTotalActions(size_t totalActions)
{
    write_lock l(lock);
    currentStatus.completedGlobalActions = 0;
    currentStatus.totalGlobalActions = totalActions;

    std::cout << "StatusTracker::setTotalActions " << totalActions << std::endl;
}

#define WRITE_LOCK write_lock l(lock);
#define READ_LOCK read_lock l(lock);

void corecvs::StatusTracker::reset(const std::string &action, size_t totalActions)
{
    WRITE_LOCK
    if (currentStatus.startedGlobalActions > 0) {
        currentStatus.completedGlobalActions++;
    }
    currentStatus.startedGlobalActions++;
    currentStatus.currentAction    = action;
    currentStatus.completedActions = currentStatus.startedActions = 0;
    currentStatus.totalActions     = totalActions;
    auto status = currentStatus;

    std::cout << "StatusTracker::reset " << status << std::endl;
}

void StatusTracker::incrementStarted()
{
    checkToCancel();    // Note: it can throw a cancelation exception

    WRITE_LOCK
    currentStatus.startedActions++;
    std::cout << "Started: " << currentStatus.startedActions << std::endl;
    auto startedActions = currentStatus.startedActions;
    auto totalActions   = currentStatus.totalActions;

    CORE_ASSERT_TRUE_S(startedActions <= totalActions);
}

void StatusTracker::incrementCompleted()
{
    if (isToCancel())
    {
        std::cout << "StatusTracker::incrementCompleted is canceling" << std::endl;
        return;
    }

    WRITE_LOCK
    currentStatus.completedActions++;
    std::cout << "StatusTracker::incrementCompleted " << currentStatus << std::endl;
    auto completedActions = currentStatus.completedActions;
    auto startedActions   = currentStatus.startedActions;
    auto totalActions     = currentStatus.totalActions;

    CORE_ASSERT_TRUE_S(completedActions <= totalActions);
    CORE_ASSERT_TRUE_S(completedActions <= startedActions);
}

AutoTracker StatusTracker::createAutoTrackerCalculationObject()
{
    return AutoTracker(this);
}

bool  corecvs::StatusTracker::isCompleted() const
{
    READ_LOCK
    auto flag = currentStatus.isCompleted;

    return flag;
}

bool  corecvs::StatusTracker::isFailed() const
{
    READ_LOCK
    auto flag = currentStatus.isFailed;

    return flag;
}

bool  corecvs::StatusTracker::isToCancel() const
{
    READ_LOCK
    auto flag = currentStatus.isToCancel;

    return flag;
}

bool  corecvs::StatusTracker::isCanceled() const
{
    READ_LOCK
    auto flag = currentStatus.isCanceled;

    return flag;
}

void corecvs::StatusTracker::setCompleted()
{
    WRITE_LOCK
    currentStatus.isCompleted = true;

    std::cout << "StatusTracker::setCompleted" << std::endl;
}

void corecvs::StatusTracker::setFailed()
{
    WRITE_LOCK
    currentStatus.isFailed = true;

    std::cout << "StatusTracker::setFailed" << std::endl;
}

void corecvs::StatusTracker::setToCancel()
{
    WRITE_LOCK
    currentStatus.isToCancel = true;

    std::cout << "StatusTracker::setToCancel" << std::endl;
}

void corecvs::StatusTracker::setCanceled()
{
    WRITE_LOCK
    currentStatus.isCanceled = true;

    std::cout << "StatusTracker::setCanceled" << std::endl;
}

void corecvs::StatusTracker::checkToCancel() const
{
    if (isToCancel())
    {
        std::cout << "StatusTracker::checkToCancel cancel_group_execution" << std::endl;
        task::self().cancel_group_execution();

        std::cout << "StatusTracker::checkToCancel throw..." << std::endl;
        throw CancelExecutionException("Cancel");
    }
}

bool corecvs::StatusTracker::isActionCompleted(const std::string &action) const
{
    READ_LOCK
    bool flag = (action == currentStatus.currentAction &&
                currentStatus.totalActions == currentStatus.completedActions);
    return flag;
}

corecvs::Status corecvs::StatusTracker::getStatus() const
{
    READ_LOCK
    return currentStatus;
}

} // namespace corecvs
