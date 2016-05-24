#ifndef APPLICATOR_H
#define APPLICATOR_H
#include <QDir>
#include <QFileInfo>

#include <iostream>
#include <string>
#include <vector>

#include <QStringList>

template <typename DerivedType>
struct Applicator
{
    class VisitableString : public std::string
    {       
    public:
        VisitableString() {}
        VisitableString(const std::string &base) : std::string(base) {}


        template<class VisitorType>
        void accept(VisitorType &visitor)
        {
            visitor.visit((std::string&)*this, std::string(""), "element");
        }
    };

    std::string destinationPath;
    std::string sourcePath;
    std::vector<VisitableString> inputFiles;

    Applicator() {}

    virtual bool applyParameters(const QStringList &params)
    {
        bool ready = true;
        QStringList inputs;
        QFileInfo sourceTemplate;

        if(params.size() < 4)
        {
            std::cerr << helpMessage() << std::endl;
            ready = false;
        }
        else
        {
            sourcePath = "";
            destinationPath = "";
            inputFiles.clear();
            sourceTemplate = QFileInfo(params.at(2));
            destinationPath = QDir(params.at(3)).absolutePath().toStdString();

            inputs = sourceTemplate.absoluteDir().entryList(QStringList() << sourceTemplate.fileName());

            if(!inputs.size())
            {
                ready = false;
                std::cerr << "Nothing to augment with template " << params.at(2).toStdString() << std::endl;
            }
        }
        if(ready)
        {
            sourcePath = sourceTemplate.absolutePath().toStdString();
            for(auto&input: inputs)
            {
                inputFiles.push_back( VisitableString(input.toStdString()));
            }
            if(params.size() > 4)
                ready = applySpecialParameters(params);
        }
        return ready;
    }

    virtual bool applySpecialParameters(const QStringList &params) = 0;

    virtual bool applyAugment(const std::string &inputFile, const std::string &outputFile) = 0;

    virtual void allApplyAugment()
    {
        QFileInfo sourceFile;
        QFileInfo destinationFile;
        QDir sourceDir(sourcePath.c_str());
        QDir destinationDir(destinationPath.c_str());
        std::cout << "Working in " << sourcePath << std::endl;
        if(!QDir(destinationPath.c_str()).exists() && !QDir().mkdir(destinationPath.c_str()))
        {
            std::cerr << "Incorrect desitnation path " << destinationPath << std::endl;
        }
        std::cout << "Saving to  " << destinationPath << std::endl;
        for(std::string&file: inputFiles)
        {
            sourceFile.setFile(sourceDir, file.c_str());
            destinationFile.setFile(destinationDir, file.c_str());
            std::cout << file
                      << (applyAugment(sourceFile.absoluteFilePath().toStdString(),
                                    destinationFile.absoluteFilePath().toStdString()) ? "[OK]" : "[FAIL]") << std::endl;
        }
    }

    virtual std::string helpMessage() = 0;

    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        ((DerivedType*)this)->accept(visitor);
    }

    template<class VisitorType>
    inline void acceptBase(VisitorType &visitor)
    {
        visitor.visit(destinationPath, std::string(""), "destinationPath");
        visitor.visit(sourcePath, std::string(""), "sourcePath");
        visitor.visit((std::vector<VisitableString>&)inputFiles, "inputFiles");
    }
};

#endif // BLURAPPLICATOR_H
