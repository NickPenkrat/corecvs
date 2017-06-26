#ifndef PARAMETERS_MAPPER_MERGER_H_
#define PARAMETERS_MAPPER_MERGER_H_
/**
 * \file parametersMapperMerger.h
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include <iostream>
#include <QtCore/QSharedPointer>

#include "reflection.h"
#include "defaultSetter.h"

#include "baseParametersControlWidget.h"
#include "mergerControlWidget.h"
#include "presentationParametersControlWidget.h"

class ParametersMapperMerger : public QObject
{
    Q_OBJECT
public:
    ParametersMapperMerger();

    void setBaseParametersControlWidget(BaseParametersControlWidget *widget);
    void setMergerControlWidget(MergerControlWidget *widget);
    void setPresentationParametersControlWidget(PresentationParametersControlWidget *widget);

public slots:
    void paramsChanged();

signals:
    void baseParametersParamsChanged(QSharedPointer<BaseParameters> params);
    void mergerParamsChanged(QSharedPointer<Merger> params);
    void presentationParametersParamsChanged(QSharedPointer<PresentationParameters> params);

private:
    BaseParametersControlWidget *mBaseParametersControlWidget;
    MergerControlWidget *mMergerControlWidget;
    PresentationParametersControlWidget *mPresentationParametersControlWidget;
};
#endif //PARAMETERS_MAPPER_MERGER_H_
