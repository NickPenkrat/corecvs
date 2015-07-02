#ifndef LENSCORRECTIONPARAMETRESCONTOLWIDGET_H
#define LENSCORRECTIONPARAMETRESCONTOLWIDGET_H

#include <QWidget>

#include "parametersControlWidgetBase.h"
#include "radialCorrection.h"
#include "advancedImageWidget.h"
#include "graphPlotDialog.h"
#include "rgb24Buffer.h"

namespace Ui {
class LensCorrectionParametresContolWidget;
}

class LensCorrectionParametresContolWidget : public ParametersControlWidgetBase
{
    Q_OBJECT

public:
    explicit LensCorrectionParametresContolWidget(QWidget *parent = 0);
    virtual ~LensCorrectionParametresContolWidget();

    LensCorrectionParametres* createParameters() const;
    void setParameters(const LensCorrectionParametres &input);
    virtual void setParametersVirtual(void *input);


    virtual void loadParamWidget(WidgetLoader &loader);
    virtual void saveParamWidget(WidgetSaver  &saver);

public slots:
    void changeParameters()
    {
        // emit paramsChanged();
    }

    void addPower();
    void delPower();
    void resetCx();
    void resetCy();
    void resetP1();
    void resetP2();


    void updateAdditionalData();

    void exampleShow();
    void loadExample();

    void showGraphDialog();

signals:
    void valueChanged();
    void paramsChanged();

private:
    Ui::LensCorrectionParametresContolWidget *ui;
    bool autoInit;
    QString rootPath;

/* Some extended data */

    GraphPlotDialog     mGraphDialog;
    AdvancedImageWidget mDemoDialog;

    QImage *mExample;

    RGB24Buffer *mInput;
    RGB24Buffer *mCorrected;
    RGB24Buffer *mInverse;
    RGB24Buffer *mBackproject;
    RGB24Buffer *mDiff;
    RGB24Buffer *mIsolines;




};

#endif // LENSCORRECTIONPARAMETRESCONTOLWIDGET_H
