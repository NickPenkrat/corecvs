#include "tempFolder.h"
#include "log.h"
#include "utils.h"
#include "folderScanner.h"

namespace corecvs {

string TempFolder::getTempFolderPath(const string &projectEnviromentVariable, bool clear)
{
    static vector< string > clearedFolders;
    string tempPath = ".";
    if (projectEnviromentVariable.empty())
    {
        L_ERROR_P("The 'projectEnviromentVariable' is empty.");
        return tempPath;
    }

    cchar* sProjectPath = std::getenv(projectEnviromentVariable.c_str());
    if (!sProjectPath)
    {
        L_ERROR_P("The <%s> enviroment variable is not set.", projectEnviromentVariable.c_str());
        return tempPath;
    }

    string projectPath = sProjectPath;
    static const char* envBuildNumber = std::getenv("BUILD_NUMBER");
    static const char* envBuildJob    = std::getenv("JOB_NAME");
    if (envBuildNumber && envBuildJob)
    {
        tempPath = projectPath + PATH_SEPARATOR + "data" + PATH_SEPARATOR + 
        "test_results" + PATH_SEPARATOR + envBuildJob + "_" + envBuildNumber + PATH_SEPARATOR + "temp";
    }
    else
    {
#ifdef WIN32
        cchar * temp = std::getenv("TEMP");
        if (temp == nullptr) temp = std::getenv("TMP");
        if (temp != nullptr)
            tempPath = temp;
        else
            L_ERROR_P("The <TEMP> enviroment variable is not set.", projectEnviromentVariable.c_str());
#else
        tempPath = "/tmp";
#endif
        tempPath += (PATH_SEPARATOR + projectEnviromentVariable);
    }
        
    bool createFolder = false;
    if (FolderScanner::isDir(tempPath))
    {
        if (clear)
        {
            bool found = false;
            for (auto dir : clearedFolders)
            {
                if (projectEnviromentVariable == dir)
                {
                    found = true;
                    break;
                }
            }

            if (!found) // delete folder to create it later
            {
#ifdef WIN32
                std::system(("rd /s /q " + tempPath).c_str());
#else
                std::system(("rm -rf " + tempPath).c_str());
#endif
                L_INFO_P("The <%s> folder is deleted.", tempPath.c_str());
                createFolder = true;
            }
        }
    }
    else
        createFolder = true;

    if (createFolder) // create folder
    {
        std::system(("mkdir " + tempPath).c_str());
        L_INFO_P("The <%s> folder is created.", tempPath.c_str());

        // the created folder is automatically considered cleared
        clearedFolders.push_back(projectEnviromentVariable);
    }    
    
    return tempPath;
}

}
