#include <QFileDialog>
#include "qtFileLoader.h"

#include "lensCorrectionParametresContolWidget.h"
#include "ui_lensCorrectionParametresContolWidget.h"
#include "displacementBuffer.h"
#include "g12Image.h"

LensCorrectionParametresContolWidget::LensCorrectionParametresContolWidget(QWidget *parent) :
    ParametersControlWidgetBase(parent),
    ui(new Ui::LensCorrectionParametresContolWidget),
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

    QObject::connect(this, SIGNAL(paramsChanged()), this, SLOT(updateAdditionalData()));

}

LensCorrectionParametresContolWidget::~LensCorrectionParametresContolWidget()
{
    delete ui;
}

void LensCorrectionParametresContolWidget::loadParamWidget(WidgetLoader &loader)
{
    LensCorrectionParametres *params = createParameters();
    loader.loadParameters(*params, rootPath);
    setParameters(*params);
    delete params;
}

void LensCorrectionParametresContolWidget::saveParamWidget(WidgetSaver  &saver)
{
    LensCorrectionParametres *params = createParameters();
    saver.saveParameters(*params, rootPath);
    delete params;
}


LensCorrectionParametres *LensCorrectionParametresContolWidget::createParameters() const
{

    /**
     * We should think of returning parameters by value or saving them in a preallocated place
     **/

    LensCorrectionParametres *result = new LensCorrectionParametres( );
    result->center.x() =  ui->centerXSpinBox->value();
    result->center.y() =  ui->centerXSpinBox->value();

    result->p1 = ui->tangential1SpinBox->value();
    result->p2 = ui->tangential2SpinBox->value();

    result->koeff.empty();
    for (int i = 0; i < ui->koefTableWidget->rowCount(); i++)
    {
        QDoubleSpinBox *box = static_cast<QDoubleSpinBox *>(ui->koefTableWidget->cellWidget(i,1));
        double value = 0.0;
        if (box != NULL) {
            QVariant str = box->value();
            value = str.toDouble();
        }
        result->koeff.push_back(value);

    }

    return result;
}

void LensCorrectionParametresContolWidget::setParameters(const LensCorrectionParametres &input)
{
    // Block signals to send them all at once
    bool wasBlocked = blockSignals(true);

    ui->centerXSpinBox->setValue(input.center.x());
    ui->centerYSpinBox->setValue(input.center.y());

    ui->tangential1SpinBox->setValue(input.p1);
    ui->tangential2SpinBox->setValue(input.p2);

    ui->koefTableWidget->clear();
    for (unsigned i = 0; i < input.koeff.size(); i++)
    {
        addPower();
    }

    for (unsigned i = 0; i < input.koeff.size(); i++)
    {

        QDoubleSpinBox *box = static_cast<QDoubleSpinBox *>(ui->koefTableWidget->cellWidget(i,1));
        box->setValue(input.koeff[i]);
    }

    blockSignals(wasBlocked);
    emit paramsChanged();
}

void LensCorrectionParametresContolWidget::setParametersVirtual(void *input)
{
    // Modify widget parameters from outside
    LensCorrectionParametres *inputCasted = static_cast<LensCorrectionParametres *>(input);
    setParameters(*inputCasted);
}


/* UI related stuff */

void LensCorrectionParametresContolWidget::addPower()
{
    int newRow = ui->koefTableWidget->rowCount() + 1;
    ui->koefTableWidget->setRowCount(newRow);
    ui->koefTableWidget->setColumnCount(2);
    ui->koefTableWidget->setItem(newRow - 1, 0, new QTableWidgetItem(QString("x^%1").arg(newRow)));

    QDoubleSpinBox *box = new QDoubleSpinBox();
    box->setDecimals(9);
    box->setSingleStep(0.00001);
    box->setMaximum(99);
    box->setMinimum(-99);
    ui->koefTableWidget->setCellWidget(newRow - 1, 1, box);

    connect(box, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    ui->koefTableWidget->setColumnWidth(0, 60);
    ui->koefTableWidget->setColumnWidth(1, 400);

}

void LensCorrectionParametresContolWidget::delPower()
{
    ui->koefTableWidget->setRowCount(ui->koefTableWidget->rowCount() - 1);
}

void LensCorrectionParametresContolWidget::resetCx()
{
    if (mExample != NULL)
    {
        ui->centerXSpinBox->setValue(mExample->width() / 2);
    } else {
        ui->centerXSpinBox->setValue(100);
    }
}

void LensCorrectionParametresContolWidget::resetCy()
{
    if (mExample != NULL)
    {
        ui->centerYSpinBox->setValue(mExample->height() / 2);
    } else {
        ui->centerYSpinBox->setValue(100);
    }
}

void LensCorrectionParametresContolWidget::resetP1()
{
    ui->tangential1SpinBox->setValue(0.0);
}

void LensCorrectionParametresContolWidget::resetP2()
{
    ui->tangential2SpinBox->setValue(0.0);
}


/* Additional stuff */
void LensCorrectionParametresContolWidget::updateAdditionalData()
{
    LensCorrectionParametres *lensParams = LensCorrectionParametresContolWidget::createParameters();
    RadialCorrection radialCorrection(*lensParams);

    int diagonal = sqrt(lensParams->center.l2Metric());
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
        qDebug() << "LensCorrectionParametresContolWidget::updateAdditionalData(): Reprocessing...";
        /* OK... */
        delete_safe(mInput);
        delete_safe(mCorrected);
        delete_safe(mInverse);
        delete_safe(mBackproject);
        delete_safe(mDiff);
        delete_safe(mIsolines);

        mInput = QTFileLoader::RGB24BufferFromQImage(mExample);

        DisplacementBuffer* mDistortionCorrectTransform =
                 DisplacementBuffer::CacheInverse(&radialCorrection,
                 mInput->h, mInput->w,
                 0.0,0.0,
                 (double)mInput->w, (double)mInput->h,
                 0.5, 0
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


void LensCorrectionParametresContolWidget::loadExample()
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

void LensCorrectionParametresContolWidget::showGraphDialog()
{
    mGraphDialog.show();
}

void LensCorrectionParametresContolWidget::exampleShow()
{
    if (mExample == NULL)
    {
        return;
    }
    switch (ui->exampleComboBox->currentIndex()) {
    case 0:
        mDemoDialog.setImage(QSharedPointer<QImage>(new QImage(*mExample)));
        break;
    case 1:
        qDebug() << "Outputing corrected";
        mDemoDialog.setImage(QSharedPointer<QImage>(new RGB24Image(mCorrected)));
        break;
    case 2:
        qDebug() << "Outputing inverse";
        mDemoDialog.setImage(QSharedPointer<QImage>(new RGB24Image(mInverse)));
        break;
    case 3:
        qDebug() << "Outputing backproject";
        mDemoDialog.setImage(QSharedPointer<QImage>(new RGB24Image(mBackproject)));
        break;
    case 4:
        qDebug() << "Outputing diff";
        mDemoDialog.setImage(QSharedPointer<QImage>(new RGB24Image(mDiff)));
        break;
    case 5:
        qDebug() << "Outputing isolines";
        mDemoDialog.setImage(QSharedPointer<QImage>(new RGB24Image(mIsolines)));
        break;
    default:
        break;
    }
    mDemoDialog.show();
}
