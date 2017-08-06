#ifndef TEMPFOLDER_H
#define TEMPFOLDER_H

#include <vector>
#include <string>

#include "global.h"

namespace corecvs {

class TempFolder {

public:

    ///
    /// \brief returns path to the project's temp folder, not a thread safe
    ///
    /// \param projectEnviromentVariable         - a unique project env var, i.e. "PROJECT_DIR"
    /// \param subfolderRelPathJen               - a subfolder relative path for Jenkins case, which will be added to get the full path to the temp folder
    /// \param clear                             - if true, temp folder is cleared when the first time method is called in a process
    ///
    /// \return project's temp folder path
    ///
    /// \note   the returned path looks like: "/tmp/projectDir" or ".../projectDir/data/test_results/master_linux_555/temp" under Jenkins
    ///
    static std::string getTempFolderPath(const std::string &projectEnviromentVariable, cchar *subfolderRelPathJen = "data/test_results", bool clear = false);

};

} // namespace corecvs

#endif // TEMPFOLDER_H
