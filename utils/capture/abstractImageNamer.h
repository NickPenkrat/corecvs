#ifndef ABSTRACTIMAGENAMER_H
#define ABSTRACTIMAGENAMER_H

#include <string>
using std::string;

class AbstractImageNamer
{
public:

    enum FileType {
        TYPE_JPEG_85,
        TYPE_JPEG_100,
        TYPE_PNG,
        TYPE_TIFF,
        TYPE_BMP,
        TYPE_UNKNOWN
    };

    static string getResolution(FileType type)
    {
        switch (type)
        {
        case TYPE_JPEG_85:
        case TYPE_JPEG_100:
            return "jpg";
        case TYPE_PNG:
            return "png";
        case TYPE_TIFF:
            return "tif";
        case TYPE_BMP:
            return "bmp";
        case TYPE_UNKNOWN:
        default:
            break;
        }
        return "";
    }

    virtual string nameForImage(const string &stName
        , int camId, string metaInfo, FileType type
        , string *path = NULL, string prefix = "", bool unique = true) = 0;
};

#endif // ABSTRACTIMAGENAMER_H
