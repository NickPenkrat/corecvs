
#pragma once
#include <QWidget>
#include <QLineEdit>
#include <QFileDialog>

#include "generatedParameters/merger.h"
#include "ui_mergerControlWidget.h"
#include "parametersControlWidgetBase.h"

namespace Ui {
    class MergerControlWidget;
}

class MergerControlWidget : public ParametersControlWidgetBase
{
    Q_OBJECT

public:
    explicit MergerControlWidget(QWidget *parent = 0, bool autoInit = false, QString rootPath = QString());
    ~MergerControlWidget();

    Merger* createParameters() const;

    void getParameters(Merger &params) const;
    void setParameters(const Merger &input);
    virtual void setParametersVirtual(void *input);


    virtual void loadParamWidget(WidgetLoader &loader);
    virtual void saveParamWidget(WidgetSaver  &saver);

    Ui::MergerControlWidget *ui()
    {
        return mUi;
    }

public slots:
    void changeParameters()
    {
        // emit paramsChanged();
    }

    void doSaveRemap();

signals:
    void valueChanged();
    void paramsChanged();
    void saveRemap(QString directory);

private:
    Ui::MergerControlWidget *mUi;
    bool autoInit;
    QString rootPath;
};


inline void MergerControlWidget::doSaveRemap()
{
    QString directoryName = QFileDialog::getExistingDirectory(
                this,
                "ChooseКак a folder",
                ".");
    if (directoryName.isNull() || directoryName.isEmpty())
    {
        return;
    }
    emit saveRemap(directoryName);
}


