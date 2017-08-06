#include "tempFolder.h"
#include "log.h"
#include "utils.h"
#include "folderScanner.h"

//#include <mutex>

using std::vector;
using std::string;

namespace corecvs {

string TempFolder::getTempFolderPath(const string &projectEnviromentVariable, cchar *subfolderRelPathJen, bool clear)
{
    static vector<string> clearedFolders;
    static string envBuildNumber = HelperUtils::getEnvVar("BUILD_NUMBER");
    static string envBuildJob    = HelperUtils::getEnvVar("JOB_NAME");

    string res = ".";
    if (projectEnviromentVariable.empty())
    {
        L_ERROR_P("The 'projectEnviromentVariable' is empty.");
        return res;
    }

    string projectPath = HelperUtils::getEnvVar(projectEnviromentVariable.c_str());
    if (projectPath.empty())
    {
        L_ERROR_P("The <%s> enviroment variable is not set.", projectEnviromentVariable.c_str());
        return res;
    }

    if (!envBuildNumber.empty() && !envBuildJob.empty())
    {
        res = projectPath;
        if (!STR_HAS_SLASH_AT_END(res)) {
            res += PATH_SEPARATOR;                  // add slash if need
        }
        if (subfolderRelPathJen && *subfolderRelPathJen) {
            res += HelperUtils::toNativeSlashes(subfolderRelPathJen);
        }
        if (!STR_HAS_SLASH_AT_END(res)) {
            res += PATH_SEPARATOR;                  // add slash if need
        }
        res += envBuildJob + "_" + envBuildNumber + PATH_SEPARATOR + "temp";
    }
    else
    {
#ifdef WIN32
        string temp = HelperUtils::getEnvVar("TEMP");
        if (temp.empty())
            temp = HelperUtils::getEnvVar("TMP");
        if (!temp.empty())
            res = temp;
        else
            L_ERROR_P("The <TEMP> enviroment variable is not set.");
#else
        res = "/tmp";
#endif
        if (!STR_HAS_SLASH_AT_END(res)) {
            res += PATH_SEPARATOR;                  // add slash if need
        }
        res += projectEnviromentVariable;
        if (STR_HAS_SLASH_AT_END(res)) {
            res.resize(res.length() - 1);           // kill the last slash
        }
    }
        
    bool createFolder = false;
    if (FolderScanner::isDir(res))                  // dir exists: clear it if others have created it
    {
        if (clear)                                  // clear it only if others have created it, we're not
        {
            if (!contains(clearedFolders, projectEnviromentVariable))
            {                                       // delete folder to create it later
#ifdef WIN32
                std::system(("rd /s /q " + res).c_str());
#else
                std::system(("rm -rf " + res).c_str());
#endif
                L_INFO_P("The <%s> folder is deleted.", res.c_str());
                createFolder = true;
            }
        }
    }
    else
    {
        createFolder = true;                        // it doesn't exist, we need to create it
    }

    if (createFolder)
    {
        FolderScanner::createDir(res);              // folder creation with its subfolders

        // the created folder is automatically considered cleared
        clearedFolders.push_back(projectEnviromentVariable);
    }    
    
    return res;
}

} // namespace corecvs
