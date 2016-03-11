#include "folderScanner.h"

#include <iostream>

#ifdef __GNUC__
# include <experimental/filesystem>
  namespace fs = std::experimental::filesystem;
#endif
#ifdef _MSC_VER
# include <filesystem>
  namespace fs = std::tr2::sys;
#endif

namespace corecvs {

bool FolderScanner::scan(const string &path, vector<string> &childs, bool findFiles)
{
    fs::path p(path);
    if (!fs::exists(p))
    {
        std::cout << p << " does not exist" << std::endl;
        return false;
    }
    if (!fs::is_directory(p))
    {
        std::cout << p << " is not a directory" << std::endl;
        return false;
    }

    for (fs::directory_iterator it = fs::directory_iterator(p); it != fs::directory_iterator(); ++it)
    {
        fs::path pathChild(*it);            // pathChild has linux style slashes inside
        bool isDir = fs::is_directory(pathChild);

        std::cout << p << " contains\t" << pathChild << " \tas a " << (isDir ? "dir" : "file") << std::endl;

        if (!(findFiles ^ isDir))
            continue;

        childs.push_back(pathChild);        // string(pathChild) has native platform style slashes inside
    }

    return true;
}

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
