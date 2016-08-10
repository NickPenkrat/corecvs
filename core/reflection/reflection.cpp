#include "reflection.h"

namespace corecvs {

/* Well... we expect some compiler magic to work and init it before the first use */
std::unique_ptr<ReflectionDirectory> ReflectionDirectoryHolder::reflectionDirectory;

ReflectionDirectory *ReflectionDirectoryHolder::getReflectionDirectory()
{
    if (!reflectionDirectory)
        reflectionDirectory.reset(new ReflectionDirectory());

    /* We lose uniquness here. That is not a big problem. Quite obvious that ReflectionDirectory has global ownership */
    return reflectionDirectory.get();
}




} // namespace corecvs

