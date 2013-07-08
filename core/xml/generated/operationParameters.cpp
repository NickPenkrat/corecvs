/**
 * \file operationParameters.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <vector>
#include <stddef.h>
#include "operationParameters.h"

/**
 *  Looks extremely unsafe because it depends on the order of static initialization.
 *  Should check standard if this is ok
 *
 *  Also it's not clear why removing "= Reflection()" breaks the code;
 **/

namespace core3vi {
template<>
Reflection BaseReflection<OperationParameters>::reflection = Reflection();
template<>
int BaseReflection<OperationParameters>::dummy = OperationParameters::staticInit();
} // namespace core3vi 

SUPPRESS_OFFSET_WARNING_BEGIN

int OperationParameters::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming(
        "Operation Parameters",
        "Operation Parameters",
        ""
    );
     

    fields().push_back(
        new EnumField
        (
          OperationParameters::OPERATION_ID,
          offsetof(OperationParameters, mOperation),
          0,
          "operation",
          "operation",
          "operation",
           NULL
        )
    );
   return 0;
}

SUPPRESS_OFFSET_WARNING_END


