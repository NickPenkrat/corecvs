#ifndef TEMPFOLDER_H
#define TEMPFOLDER_H

#include <vector>
#include <string>

#include "global.h"

using std::vector;
using std::string;

namespace corecvs {

class TempFolder {

public:

    ///
    /// \brief returns path to the project's temp folder, not a thread safe
    /// \param projectEnviromentVariable         - a unique project env var, i.e. "TOPCON_DIR"
    /// \param clear                             - if true, temp folder is cleared the first time method is called in a process
    ///
    /// \return project's temp folder path
    ///
    static string getTempFolderPath(const string &projectEnviromentVariable, bool clear = true);

};

} // namespace corecvs

#endif // TEMPFOLDER_H
