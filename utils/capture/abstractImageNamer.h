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
        TYPE_RAW10BIT,
        TYPE_UNKNOWN
    };

    static string getExtension(FileType type)
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
        case TYPE_RAW10BIT:
            return "raw";
        case TYPE_UNKNOWN:
        default:
            break;
        }
        return "";
    }

    virtual string nameForImage(const string &stId
        , const string &camId
        , string        metaInfo
        , FileType      type
        , string       *path = NULL
        , string        prefix = ""
        , bool          unique = true) const = 0;
};

#endif // ABSTRACTIMAGENAMER_H
