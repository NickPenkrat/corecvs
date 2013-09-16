#ifndef RECORDER_H_
#define RECORDER_H_
/**
 * \file recorder.h
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include "reflection.h"
#include "defaultSetter.h"
#include "printerVisitor.h"

/*
 *  Embed includes.
 */
/*
 *  Additional includes for Composite Types.
 */

using namespace corecvs;

/*
 *  Additional includes for Pointer Types.
 */

namespace corecvs {
}
/*
 *  Additional includes for enum section.
 */

/**
 * \brief Recorder parameters 
 * Recorder parameters 
 **/
class Recorder : public BaseReflection<Recorder>
{
public:
    enum FieldId {
        PATH_ID,
        FILETEMPLATE_ID,
        RECORDER_FIELD_ID_NUM
    };

    /** Section with variables */

    /** 
     * \brief path 
     * path 
     */
    std::string mPath;

    /** 
     * \brief fileTemplate 
     * fileTemplate 
     */
    std::string mFileTemplate;

    /** Static fields init function, this is used for "dynamic" field initialization */ 
    static int staticInit();

    /** Section with getters */
    const void *getPtrById(int fieldId) const
    {
        return (const unsigned char *)(this) + fields()[fieldId]->offset;
    }
    std::string path() const
    {
        return mPath;
    }

    std::string fileTemplate() const
    {
        return mFileTemplate;
    }

    /* Section with setters */
    void setPath(std::string path)
    {
        mPath = path;
    }

    void setFileTemplate(std::string fileTemplate)
    {
        mFileTemplate = fileTemplate;
    }

    /* Section with embedded classes */
    /* visitor pattern - http://en.wikipedia.org/wiki/Visitor_pattern */
template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        visitor.visit(mPath,                      static_cast<const StringField *>  (fields()[PATH_ID]));
        visitor.visit(mFileTemplate,              static_cast<const StringField *>  (fields()[FILETEMPLATE_ID]));
    }

    Recorder()
    {
        DefaultSetter setter;
        accept(setter);
    }

    Recorder(
          std::string path
        , std::string fileTemplate
    )
    {
        mPath = path;
        mFileTemplate = fileTemplate;
    }

    friend ostream& operator << (ostream &out, Recorder &toSave)
    {
        PrinterVisitor printer(out);
        toSave.accept<PrinterVisitor>(printer);
        return out;
    }

    void print ()
    {
        cout << *this;
    }
};
#endif  //RECORDER_H_
