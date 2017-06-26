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

#include <sstream>

#include "utils.h"
#include <sstream>


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

std::string removeLeading(const string &str, const string &symbols)
{
    const auto strBegin = str.find_first_not_of(symbols);
    if (strBegin == std::string::npos)
        return std::string(); // no content

    return str.substr(strBegin);
}

bool startsWith(const string &str, const string &prefix)
{
    return (str.compare(0, prefix.size(), prefix) == 0);
}

bool endsWith(const std::string &str, const std::string &postfix)
{
    if (str.length() < postfix.length())
        return false;
    return (str.compare(str.length() - postfix.length(), postfix.length(), postfix) == 0);
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

string getEnvVar(cchar *envVarName)
{
    cchar* var = std::getenv(envVarName);
    if (var == NULL || var[0] == 0) {
        return "";
    }

    return var;
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

void stringSplit(const string &s, char delim, std::vector<string> &elems)
{
    std::stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
}

std::vector<string> stringSplit(const string &s, char delim)
{
    std::vector<string> elems;
    stringSplit(s, delim, elems);
    return elems;
}

std::string stringCombine(std::vector<std::string> parts, char delim)
{
    std::string toReturn;
    for(size_t i = 0; i < parts.size(); i++)
    {
        if (i != 0) {
            toReturn += delim;
        }
        toReturn.append(parts[i]);
    }
    return toReturn;
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

string escapeString(const string &s, const std::unordered_map<char, char> &symbols, const string &escape)
{
     std::ostringstream out;

     for (const char &symbol : s)
     {

         auto p = symbols.find(symbol);
         if (p != symbols.end())
         {
             out << escape;
             out << p->second;
         } else {
             out << symbol;
         }
     }
     return out.str();
}

string unescapeString(const string &s, const std::unordered_map<char, char> &symbols, char guard)
{
    std::ostringstream out;
    for (size_t n = 0; n < s.size(); n++)
    {
        if (s[n] == guard && n < s.size() - 1)
        {
            auto p = symbols.find(s[n + 1]);
            if (p != symbols.end())
            {
                out << p->second;
            } else {
                out << guard;
            }
            n++;
        } else {
            out << s[n];
        }
    }
    return out.str();
}

} // namespace HelperUtils

} //namespace corecvs


#if defined(DSP_TARGET) || defined(WIN32) || defined(WIN64)

    // It is possible but quite hard and usually not needed to print stack trace on Win32.
    // Debugging shall be done with debugger when possible, if not, minidump is better than stack trace
    // http://stackoverflow.com/questions/105659/how-can-one-grab-a-stack-trace-in-c/127012#127012
    //
    void setStdTerminateHandler() {}

#else

static void stdTerminateHandler()
{
    printStackTrace();
    exit(1);
}

void setStdTerminateHandler()
{
    std::set_terminate(stdTerminateHandler);
}

#endif // DSP_TARGET
