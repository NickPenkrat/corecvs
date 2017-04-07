#include "folderScanner.h"
#include "log.h"
#include "utils.h"

#include <iostream>

#ifdef __GNUC__
#ifdef CORE_UNSAFE_DEPS
# include <experimental/filesystem>
  namespace fs = std::experimental::filesystem;  
#else
    #include <sys/types.h>
    #include <dirent.h>
#endif
#endif


#ifdef _MSC_VER
# include <filesystem>
  namespace fs = std::tr2::sys;
#endif

namespace corecvs {


#if defined(CORE_UNSAFE_DEPS) || defined (_MSC_VER)

bool FolderScanner::isDir(const string &path)
{
    fs::path p(path);
    return fs::exists(p) && fs::is_directory(p);
}

bool FolderScanner::createDir(const string &path)
{
    if (isDir(path))
        return true;
    std::cout << "creating dir <" << path << ">" << std::endl;

    fs::path p(path);
    return fs::create_directory(p);
}

bool FolderScanner::scan(const string &path, vector<string> &childs, bool findFiles)
{
    if (!isDir(path))
    {
        L_ERROR_P("<%s> does not exist or not a directory", path.c_str());
        return false;
    }

    fs::path p(path);
    for (fs::directory_iterator it = fs::directory_iterator(p); it != fs::directory_iterator(); ++it)
    {
        fs::path pathChild(*it);            // pathChild has linux style slashes inside

        bool isDir = fs::is_directory(pathChild);

        L_DDEBUG_P("%s contains\t%s\tas a %s", p.string().c_str(), pathChild.string().c_str(), (isDir ? "dir" : "file"));

        if (!(findFiles ^ isDir))
            continue;

        // win32: bugfix state:
        //  pathChild.string() - has linux   style slashes everywhere
        //  (string)pathChild  - has windows style slashes on vc12, but it's obsolete in vc14
#ifdef _MSC_VER
        childs.push_back(corecvs::HelperUtils::toNativeSlashes(pathChild.string()));
#else
        childs.push_back(pathChild);
#endif
    }

    return true;
}

#else

bool FolderScanner::isDir(const string &path)
{
    DIR *dp = opendir(path.c_str());
    if (dp == NULL)
        return false;

    closedir(dp);
    return true;
}

bool FolderScanner::createDir(const string &path)
{
    if (isDir(path))
        return true;
    std::cout << "creating dir <" << path << ">" << std::endl;

    std::system(("mkdir " + path).c_str());
}

bool FolderScanner::scan(const string &path, vector<string> &childs, bool findFiles)
{
    if (!isDir(path))
    {
        L_ERROR_P("<%s> does not exist or not a directory", path.c_str());
        return false;
    }

    DIR *dp = opendir(path.c_str());  CORE_ASSERT_TRUE_S(dp != NULL);

    struct dirent *ep;
    while ((ep = readdir(dp)) != NULL)
    {
        /* We need to form path */
        string childPath = path + PATH_SEPARATOR + ep->d_name;

        /* Ok there are devices, pipes, links... I don't know... */
        bool dir = (ep->d_type != DT_REG) && (ep->d_type != DT_LNK);
        if (ep->d_type == DT_UNKNOWN)
        {
            dir = isDir(childPath);
        }

        L_DDEBUG_P("%s contains\t%s\tas a %s (d_type:0x%x)", path.c_str(), ep->d_name, (dir ? "dir" : "file"), ep->d_type);

        if (!(findFiles ^ dir))
            continue;

        childs.push_back(childPath);
    }

    closedir(dp);
    return true;
}

#endif


} // namespace corecvs


/*
int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cout << "Second arg is path" << std::endl;
        exit(-1);
    }
    fs::path p(argv[1]);
    if (!fs::exists(p))
    {
        std::cout << p << " does not exist" << std::endl;
        exit(-2);
    }
    if (!fs::is_directory(p))
    {
        std::cout << p << " is not a directory" << std::endl;
        exit(-3);
    }
    fs::directory_iterator d(p);

    for (fs::directory_iterator d = fs::directory_iterator(p); d != fs::directory_iterator(); ++d)
    {
        fs::path p_child(*d);
        std::cout << p << " contains " << p_child << " as a " << (fs::is_directory(p_child) ? "dir" : "file") << std::endl;
    }
    return 0;
}
*/
