#ifndef BLURAPPLICATOR_H
#define BLURAPPLICATOR_H
#include "applicator.h"

#include <string>
#include <vector>

#include <QStringList>


struct BlurApplicator: Applicator<BlurApplicator>
{
    int kernelSize = 3;

    BlurApplicator();

    virtual bool applySpecialParameters(const QStringList &params);

    virtual bool applyAugment(const std::string &inputFile, const std::string &outputFile);

    virtual std::string helpMessage();

    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        acceptBase(visitor);
        visitor.visit(kernelSize, 3, "kernelSize");
    }


};

#endif // BLURAPPLICATOR_H
