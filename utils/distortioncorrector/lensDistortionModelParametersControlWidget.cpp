#include <QFileDialog>
#include "qtFileLoader.h"

#include "lensDistortionModelParametersControlWidget.h"
#include "ui_lensDistortionModelParametersControlWidget.h"
#include "displacementBuffer.h"
#include "g12Image.h"

LensDistortionModelParametersControlWidget::LensDistortionModelParametersControlWidget(QWidget *parent) :
    ParametersControlWidgetBase(parent),
    ui(new Ui::LensDistortionModelParametersContolWidget),
    mExample(NULL),
    mInput(NULL),
    mCorrected(NULL),
    mInverse(NULL),
    mBackproject(NULL),
    mDiff(NULL),
    mIsolines(NULL)
{
    ui->setupUi(this);

    QObject::connect(ui->centerXSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(ui->centerYSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));

    QObject::connect(ui->tangential1SpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(ui->tangential2SpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));

    QObject::connect(ui->koefTableWidget, SIGNAL(itemChanged(QTableWidgetItem*)), this, SIGNAL(paramsChanged()));

    QObject::connect(ui->freeParamSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));


    QObject::connect(ui->loadPushButton, SIGNAL(released()), this, SLOT(loadParams()));
    QObject::connect(ui->savePushButton, SIGNAL(released()), this, SLOT(saveParams()));

    QObject::connect(this, SIGNAL(paramsChanged()), this, SLOT(updateAdditionalData()));

}

LensDistortionModelParametersControlWidget::~LensDistortionModelParametersControlWidget()
{
    delete ui;
}

void LensDistortionModelParametersControlWidget::loadParamWidget(WidgetLoader &loader)
{
    LensDistortionModelParameters *params = createParameters();
    loader.loadParameters(*params, rootPath);
    setParameters(*params);
    delete params;
}

void LensDistortionModelParametersControlWidget::saveParamWidget(WidgetSaver  &saver)
{
    LensDistortionModelParameters *params = createParameters();
    saver.saveParameters(*params, rootPath);
    delete params;
}


LensDistortionModelParameters *LensDistortionModelParametersControlWidget::createParameters() const
{

    /**
     * We should think of returning parameters by value or saving them in a preallocated place
     **/

    LensDistortionModelParameters *result = new LensDistortionModelParameters( );
    result->setPrincipalX(ui->centerXSpinBox->value());
    result->setPrincipalY(ui->centerYSpinBox->value());

    result->setTangentialX(ui->tangential1SpinBox->value());
    result->setTangentialY(ui->tangential2SpinBox->value());

    result->mKoeff.empty();
    for (int i = 0; i < ui->koefTableWidget->rowCount(); i++)
    {
        QDoubleSpinBox *box = static_cast<QDoubleSpinBox *>(ui->koefTableWidget->cellWidget(i,COLUMN_EDIT));
        double value = 0.0;
        if (box != NULL) {
            QVariant str = box->value();
            value = str.toDouble();
        }
        result->mKoeff.push_back(value);

    }

    return result;
}

void LensDistortionModelParametersControlWidget::setParameters(const LensDistortionModelParameters &input)
{
    // Block signals to send them all at once
    bool wasBlocked = blockSignals(true);

    ui->centerXSpinBox->setValue(input.principalX());
    ui->centerYSpinBox->setValue(input.principalY());

    ui->tangential1SpinBox->setValue(input.tangentialX());
    ui->tangential2SpinBox->setValue(input.tangentialY());

    ui->koefTableWidget->setRowCount(0);
    for (unsigned i = 0; i < input.mKoeff.size(); i++)
    {
        addPower();
    }

    for (unsigned i = 0; i < input.mKoeff.size(); i++)
    {
        QWidget* widget = ui->koefTableWidget->cellWidget(i,COLUMN_EDIT);
        if (widget == NULL)
        {
            qDebug() << "LensDistortionModelParametersControlWidget::setParameters(): we have internal problem";
            break;
        }
        QDoubleSpinBox *box = static_cast<QDoubleSpinBox *>(widget);
        box->setValue(input.mKoeff[i]);
    }

    blockSignals(wasBlocked);
    emit paramsChanged();
}

void LensDistortionModelParametersControlWidget::setParametersVirtual(void *input)
{
    // Modify widget parameters from outside
    LensDistortionModelParameters *inputCasted = static_cast<LensDistortionModelParameters *>(input);
    setParameters(*inputCasted);
}


/* UI related stuff */

void LensDistortionModelParametersControlWidget::addPower()
{
    int newRow = ui->koefTableWidget->rowCount() + 1;
    ui->koefTableWidget->setRowCount(newRow);
    ui->koefTableWidget->setColumnCount(2);
    ui->koefTableWidget->setItem(newRow - 1, COLUMN_CAPTION, new QTableWidgetItem(QString("x^%1").arg(newRow)));

    QDoubleSpinBox *box = new QDoubleSpinBox();
    box->setDecimals(9);
    box->setSingleStep(0.00001);
    box->setMaximum(99);
    box->setMinimum(-99);
    ui->koefTableWidget->setCellWidget(newRow - 1, COLUMN_EDIT, box);

    connect(box, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    ui->koefTableWidget->setColumnWidth(0, 60);
    ui->koefTableWidget->setColumnWidth(1, 250);

}

void LensDistortionModelParametersControlWidget::delPower()
{
    ui->koefTableWidget->setRowCount(ui->koefTableWidget->rowCount() - 1);
}

void LensDistortionModelParametersControlWidget::resetCx()
{
    if (mExample != NULL)
    {
        ui->centerXSpinBox->setValue(mExample->width() / 2);
    } else {
        ui->centerXSpinBox->setValue(100);
    }
}

void LensDistortionModelParametersControlWidget::resetCy()
{
    if (mExample != NULL)
    {
        ui->centerYSpinBox->setValue(mExample->height() / 2);
    } else {
        ui->centerYSpinBox->setValue(100);
    }
}

void LensDistortionModelParametersControlWidget::resetP1()
{
    ui->tangential1SpinBox->setValue(0.0);
}

void LensDistortionModelParametersControlWidget::resetP2()
{
    ui->tangential2SpinBox->setValue(0.0);
}


/* Additional stuff */
void LensDistortionModelParametersControlWidget::updateAdditionalData()
{
    LensDistortionModelParameters *lensParams = LensDistortionModelParametersControlWidget::createParameters();
    RadialCorrection radialCorrection(*lensParams);

    Vector2dd principal(lensParams->principalX(), lensParams->principalY());

    int diagonal = sqrt(principal.l2Metric());
    if (mExample != NULL)
    {
        diagonal = Vector2dd(mExample->width(), mExample->height()).l2Metric();
    }

    for (int i = 0; i < 20; i++)
    {
        mGraphDialog.addGraphPoint(0, 0, false);
    }

    for (int i = 0; i < diagonal; i++)
    {
        mGraphDialog.addGraphPoint(0, radialCorrection.radialScale((double)i), true);
    }
    mGraphDialog.update();

    PrinterVisitor printer;
    printer.visit(*lensParams, "inputPrams");


    if (mExample != NULL)
    {
        qDebug() << "LensDistortionModelParametersContolWidget::updateAdditionalData(): Reprocessing...";
        /* OK... */
        delete_safe(mInput);
        delete_safe(mCorrected);
        delete_safe(mInverse);
        delete_safe(mBackproject);
        delete_safe(mDiff);
        delete_safe(mIsolines);

        mInput = QTFileLoader::RGB24BufferFromQImage(mExample);

        double step = ui->freeParamSpinBox->value();
        if (step < 0.05)
        {
            step = 0.05;
        }

        DisplacementBuffer* mDistortionCorrectTransform =
                 DisplacementBuffer::CacheInverse(&radialCorrection,
                 mInput->h, mInput->w,
                 0.0,0.0,
                 (double)mInput->w, (double)mInput->h,
                 step, 0
        );

        //DisplacementBuffer* mDistortionCorrectTransform = DisplacementBuffer::TestWiggle(mInput->h, mInput->w);

        mCorrected = mInput->doReverseDeformation<RGB24Buffer, DisplacementBuffer>(
                    *mDistortionCorrectTransform,
                    mInput->h, mInput->w
                );
        mCorrected->drawCrosshare1(10,10, RGBColor::Red());


        mInverse = mInput->doReverseDeformation<RGB24Buffer, RadialCorrection>(
                    radialCorrection,
                    mInput->h, mInput->w
                );
        mInverse->drawCrosshare1(10,10, RGBColor::Green());


        mBackproject = mCorrected->doReverseDeformation<RGB24Buffer, RadialCorrection>(
                    radialCorrection,
                    mCorrected->h, mCorrected->w
                );
        mBackproject->drawCrosshare1(10,10, RGBColor::Yellow());

        Map2DFunction<DisplacementBuffer> mapFunc(mDistortionCorrectTransform);
        LengthFunction lengthFunc(&mapFunc);

        mIsolines = new RGB24Buffer(mDistortionCorrectTransform->getSize());
        mIsolines->drawIsolines(0.0, 0.0,
                (double)mDistortionCorrectTransform->h,  (double)mDistortionCorrectTransform->w,
                0.1, lengthFunc);

        mDiff = new RGB24Buffer(mInput->getSize());
        for (int i = 0; i < mInput->h; i++)
        {
            for (int j = 0; j < mInput->w; j++)
            {
                mDiff->element(i,j) = RGBColor::diff(mInput->element(i,j),mBackproject->element(i,j));
            }
        }


        delete_safe(mDistortionCorrectTransform);

        /* FINAL */

        exampleShow();
    }


    delete lensParams;
}


void LensDistortionModelParametersControlWidget::loadExample()
{
    QString filename = QFileDialog::getOpenFileName(
        this,
        "Choose an file name",
        ".",
        "Image (*.jpg *.jpeg *.bmp *.pgm *.png)"
        );


    QImage *image = new QImage(filename);
    if (image->isNull())
    {
        delete image;
        return;
    }

    delete_safe(mExample);
    mExample = image;
    updateAdditionalData();
}

void LensDistortionModelParametersControlWidget::showGraphDialog()
{
    mGraphDialog.show();
}

void LensDistortionModelParametersControlWidget::loadParams()
{
    qDebug() << "LensDistortionModelParametersControlWidget::loadParams(): called";
    QString filename =  QFileDialog::getOpenFileName(
                this,
                "Choose an file name",
                ".",
                "json distortion params (*.json)"
                );

    {
        JSONGetter getter(filename);
        WidgetLoader loader(&getter);
        loadParamWidget(loader);
    }
}

void LensDistortionModelParametersControlWidget::saveParams()
{
    qDebug() << "LensDistortionModelParametersControlWidget::saveParams(): called";
    QString filename =  QFileDialog::getSaveFileName(
                this,
                "Choose an file name",
                ".",
                "json distortion params (*.json)"
                );

    {
        JSONSetter setter(filename);
        WidgetSaver saver(&setter);
        saveParamWidget(saver);
    }

}

void LensDistortionModelParametersControlWidget::exampleShow()
{
    if (mExample == NULL)
    {
        return;
    }
    switch (ui->exampleComboBox->currentIndex()) {
    case 0:
        mDemoDialog.setImage(QSharedPointer<QImage>(new QImage(*mExample)));
        mDemoDialog.setWindowTitle("Example Input Image");
        break;
    case 1:
        qDebug() << "Outputing corrected";
        mDemoDialog.setImage(QSharedPointer<QImage>(new RGB24Image(mCorrected)));
        mDemoDialog.setWindowTitle("Corrected Output");
        break;
    case 2:
        qDebug() << "Outputing inverse";
        mDemoDialog.setImage(QSharedPointer<QImage>(new RGB24Image(mInverse)));
        mDemoDialog.setWindowTitle("Inverse Corrected");
        break;
    case 3:
        qDebug() << "Outputing backproject";
        mDemoDialog.setImage(QSharedPointer<QImage>(new RGB24Image(mBackproject)));
        mDemoDialog.setWindowTitle("Backproject Output");
        break;
    case 4:
        qDebug() << "Outputing diff";
        mDemoDialog.setImage(QSharedPointer<QImage>(new RGB24Image(mDiff)));
        mDemoDialog.setWindowTitle("Difference");
        break;
    case 5:
        qDebug() << "Outputing isolines";
        mDemoDialog.setImage(QSharedPointer<QImage>(new RGB24Image(mIsolines)));
        mDemoDialog.setWindowTitle("Isolines view");
        break;
    default:
        break;
    }
    mDemoDialog.show();
}