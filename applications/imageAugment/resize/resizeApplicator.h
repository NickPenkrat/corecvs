#ifndef RESIZEAPPLICATOR_H
#define RESIZEAPPLICATOR_H
#include "applicator.h"

struct ResizeApplicator: Applicator<ResizeApplicator>
{
    double scale = 0.5;

    ResizeApplicator();

    virtual bool applySpecialParameters(const QStringList &params);

    virtual bool applyAugment(const std::string &inputFile, const std::string &outputFile);

    virtual std::string helpMessage();

    template<class VisitorType>
    void accept(VisitorType &visitor)
    {
        acceptBase(visitor);
        visitor.visit(scale, 0.5, "scale");
    }
};

#endif // RESIZEAPPLICATOR_H
