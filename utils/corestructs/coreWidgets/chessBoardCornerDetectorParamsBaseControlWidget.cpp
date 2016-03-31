/**
 * \file chessBoardCornerDetectorParamsBaseControlWidget.cpp
 * \attention This file is automatically generated and should not be in general modified manually
 *
 * \date MMM DD, 20YY
 * \author autoGenerator
 */

#include "chessBoardCornerDetectorParamsBaseControlWidget.h"
#include "ui_chessBoardCornerDetectorParamsBaseControlWidget.h"
#include "qSettingsGetter.h"
#include "qSettingsSetter.h"


ChessBoardCornerDetectorParamsBaseControlWidget::ChessBoardCornerDetectorParamsBaseControlWidget(QWidget *parent, bool _autoInit, QString _rootPath)
    : ParametersControlWidgetBase(parent)
    , mUi(new Ui::ChessBoardCornerDetectorParamsBaseControlWidget)
    , autoInit(_autoInit)
    , rootPath(_rootPath)
{
    mUi->setupUi(this);

    QObject::connect(mUi->gradientCrossWidthSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->sectorSizeDegSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->histogramBinsSpinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->minAngleDegSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->neighborhoodSpinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->gradThresholdSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->orientationInlierThresholdSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->inlierDistanceThresholdSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->updateThresholdSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->scoreThresholdSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->nRoundsSpinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->meanshiftBandwidthSpinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->nmsLocalitySpinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->patternRadius, SIGNAL(valueChanged()), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->patternStartAngleDeg, SIGNAL(valueChanged()), this, SIGNAL(paramsChanged()));
    QObject::connect(mUi->cornerScores, SIGNAL(valueChanged()), this, SIGNAL(paramsChanged()));
}

ChessBoardCornerDetectorParamsBaseControlWidget::~ChessBoardCornerDetectorParamsBaseControlWidget()
{

    delete mUi;
}

void ChessBoardCornerDetectorParamsBaseControlWidget::loadParamWidget(WidgetLoader &loader)
{
    ChessBoardCornerDetectorParamsBase *params = createParameters();
    loader.loadParameters(*params, rootPath);
    setParameters(*params);
    delete params;
}

void ChessBoardCornerDetectorParamsBaseControlWidget::saveParamWidget(WidgetSaver  &saver)
{
    ChessBoardCornerDetectorParamsBase *params = createParameters();
    saver.saveParameters(*params, rootPath);
    delete params;
}

 /* Composite fields are NOT supported so far */
void ChessBoardCornerDetectorParamsBaseControlWidget::getParameters(ChessBoardCornerDetectorParamsBase& params) const
{

    params.setGradientCrossWidth(mUi->gradientCrossWidthSpinBox->value());
    params.setSectorSizeDeg    (mUi->sectorSizeDegSpinBox->value());
    params.setHistogramBins    (mUi->histogramBinsSpinBox->value());
    params.setMinAngleDeg      (mUi->minAngleDegSpinBox->value());
    params.setNeighborhood     (mUi->neighborhoodSpinBox->value());
    params.setGradThreshold    (mUi->gradThresholdSpinBox->value());
    params.setOrientationInlierThreshold(mUi->orientationInlierThresholdSpinBox->value());
    params.setInlierDistanceThreshold(mUi->inlierDistanceThresholdSpinBox->value());
    params.setUpdateThreshold  (mUi->updateThresholdSpinBox->value());
    params.setScoreThreshold   (mUi->scoreThresholdSpinBox->value());
    params.setNRounds          (mUi->nRoundsSpinBox->value());
    params.setMeanshiftBandwidth(mUi->meanshiftBandwidthSpinBox->value());
    params.setNmsLocality      (mUi->nmsLocalitySpinBox->value());
    params.setPatternRadius    (mUi->patternRadius->value());
    params.setPatternStartAngleDeg(mUi->patternStartAngleDeg->value());
    params.setCornerScores     (mUi->cornerScores->value());

}

ChessBoardCornerDetectorParamsBase *ChessBoardCornerDetectorParamsBaseControlWidget::createParameters() const
{

    /**
     * We should think of returning parameters by value or saving them in a preallocated place
     **/


    ChessBoardCornerDetectorParamsBase *result = new ChessBoardCornerDetectorParamsBase(
          mUi->gradientCrossWidthSpinBox->value()
        , mUi->sectorSizeDegSpinBox->value()
        , mUi->histogramBinsSpinBox->value()
        , mUi->minAngleDegSpinBox->value()
        , mUi->neighborhoodSpinBox->value()
        , mUi->gradThresholdSpinBox->value()
        , mUi->orientationInlierThresholdSpinBox->value()
        , mUi->inlierDistanceThresholdSpinBox->value()
        , mUi->updateThresholdSpinBox->value()
        , mUi->scoreThresholdSpinBox->value()
        , mUi->nRoundsSpinBox->value()
        , mUi->meanshiftBandwidthSpinBox->value()
        , mUi->nmsLocalitySpinBox->value()
        , mUi->patternRadius->value()
        , mUi->patternStartAngleDeg->value()
        , mUi->cornerScores->value()
    );
    return result;
}

void ChessBoardCornerDetectorParamsBaseControlWidget::setParameters(const ChessBoardCornerDetectorParamsBase &input)
{
    // Block signals to send them all at once
    bool wasBlocked = blockSignals(true);
    mUi->gradientCrossWidthSpinBox->setValue(input.gradientCrossWidth());
    mUi->sectorSizeDegSpinBox->setValue(input.sectorSizeDeg());
    mUi->histogramBinsSpinBox->setValue(input.histogramBins());
    mUi->minAngleDegSpinBox->setValue(input.minAngleDeg());
    mUi->neighborhoodSpinBox->setValue(input.neighborhood());
    mUi->gradThresholdSpinBox->setValue(input.gradThreshold());
    mUi->orientationInlierThresholdSpinBox->setValue(input.orientationInlierThreshold());
    mUi->inlierDistanceThresholdSpinBox->setValue(input.inlierDistanceThreshold());
    mUi->updateThresholdSpinBox->setValue(input.updateThreshold());
    mUi->scoreThresholdSpinBox->setValue(input.scoreThreshold());
    mUi->nRoundsSpinBox->setValue(input.nRounds());
    mUi->meanshiftBandwidthSpinBox->setValue(input.meanshiftBandwidth());
    mUi->nmsLocalitySpinBox->setValue(input.nmsLocality());
    mUi->patternRadius->setValue(input.patternRadius());
    mUi->patternStartAngleDeg->setValue(input.patternStartAngleDeg());
    mUi->cornerScores->setValue(input.cornerScores());
    blockSignals(wasBlocked);
    emit paramsChanged();
}

void ChessBoardCornerDetectorParamsBaseControlWidget::setParametersVirtual(void *input)
{
    // Modify widget parameters from outside
    ChessBoardCornerDetectorParamsBase *inputCasted = static_cast<ChessBoardCornerDetectorParamsBase *>(input);
    setParameters(*inputCasted);
}
