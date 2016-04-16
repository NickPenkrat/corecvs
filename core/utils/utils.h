#ifndef UTILS_H_
#define UTILS_H_
/**
 * \file utils.h
 * \brief This calss conatins CPP helper functions
 *
 *
 *
 * \ingroup cppcorefiles
 * \date Apr 14, 2010
 * \author alexander
 */

#include <string>
#include <iostream>

#include "global.h"


namespace corecvs {

//@{
/**
 * This is a helper meta-programming template to represent binary constants.
 *
 * For example:
 *         9 == binary<1001>::value
 *
 *
 **/
template <unsigned long int N>
class binary
{
public:
    const static unsigned value = (binary<N / 10>::value * 2) + ((N % 10 == 0) ? 0 : 1);
};

template <>
class binary<0>
{
    const static unsigned value = 0;
};

//@}

/** Useful class of some tasty things
 */
namespace HelperUtils
{
    using std::string;
    using std::istream;

    istream&        getlineSafe(istream& is, string& str);

    string          toNativeSlashes(const string& str);

    string          getEnvDirPath(cchar *envVarName);
    string          getFullPath(const string& envDirPath, cchar* path, cchar* filename = NULL);

    inline string   getFullPath(cchar *envVarName, cchar* path, cchar* filename = NULL)
    {
        return  getFullPath(getEnvDirPath(envVarName), path, filename);
    }

    inline string   getFileNameFromFilePath(const string &filePath)
    {
        return filePath.substr(filePath.find_last_of("/\\") + 1);
    }

    inline string   getPathWithoutFilename(const string &filePath)
    {
        return filePath.substr(0, filePath.find_last_of("/\\") + 1);
    }

    inline string   getFullPathWithoutExt(const string &filePath)
    {
        return filePath.substr(0, filePath.find_last_of("."));
    }

    /// Add suffix to file name before extension: filename.ext -> filenameSuffix.ext
    inline string   getFilePathWithSuffixAtName(const string& filePath, const string& suffix)
    {
        size_t extPosition = filePath.find_last_of(".");
        return filePath.substr(0, extPosition) + suffix + filePath.substr(extPosition + 1);
    }

    inline bool     startsWith(const string &str, const string &prefix)
    {
        return (str.compare(0, prefix.size(), prefix) == 0);
    }


} // namespace HelperUtils


/* OS related stuff */
void setStdTerminateHandler();

} //namespace corecvs

#endif /* UTILS_H_ */
