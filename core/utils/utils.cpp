/**
 * \file utils.cpp
 * \brief Add Comment Here
 *
 * \ingroup cppcorefiles
 * \date Apr 14, 2010
 * \author alexander
 */

#if !defined( DSP_TARGET ) && !defined( WIN32 ) && !defined( WIN64 )

#include <iostream>
#include <exception>
#include <cstdlib>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <execinfo.h>

#endif

#include "utils.h"


namespace corecvs {

namespace HelperUtils {

istream &getlineSafe(istream &is, string &str)
{
    istream& stream = getline(is, str);

    if (str.length() != 0 &&  *str.rbegin() == '\r') {
        str.erase(str.length() - 1);
    }

    return stream;
}

string getEnvDirPath(cchar *envVarName)
{
    cchar* dir = std::getenv(envVarName);
    if (dir == NULL || dir[0] == 0) {
        CORE_ASSERT_FAIL_P(("Missed environment variable %s", envVarName));
        return "";
    }

    string toReturn(dir);
    if (!STR_HAS_SLASH_AT_END(toReturn)) {
        toReturn += PATH_SEPARATOR;
    }
    return toReturn;
}

static string replaceSlashes(const string& str, const string& oldStr, const string& newStr)
{
    size_t pos = 0;
    string s = str;
    while ((pos = str.find(oldStr, pos)) != string::npos)
    {
        s.replace(pos, oldStr.length(), newStr);
        pos += newStr.length();
    }
    return s;
}

string toNativeSlashes(const string& str)
{
#ifdef WIN32
    return replaceSlashes(str, "/", PATH_SEPARATOR);
#else
    return replaceSlashes(str, "\\", PATH_SEPARATOR);
#endif
}

string getFullPath(const string& envDirPath, cchar* path, cchar* filename)
{
    CORE_ASSERT_TRUE_S(path != NULL);

    if (filename == NULL)
        filename = "";

    if (envDirPath.empty())
        return filename;

    return toNativeSlashes(envDirPath + path + filename);
}

} // namespace HelperUtils

} //namespace corecvs


#if defined(DSP_TARGET) || defined(WIN32) || defined(WIN64)

    // It is possible but quite hard and usually not needed to print stack trace on Win32.
    // Debugging shall be done with debugger when possible, if not, minidump is better than stack trace
    // http://stackoverflow.com/questions/105659/how-can-one-grab-a-stack-trace-in-c/127012#127012
    //
    void setHandlerStdTerminate() {}

#else

static void stdTerminateHandler()
{
    printStackTrace();
    exit(1);
}

void setHandlerStdTerminate()
{
    std::set_terminate(stdTerminateHandler);
}

#endif // DSP_TARGET
