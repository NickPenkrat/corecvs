#include "shadedSceneControlWidget.h"
#include "ui_shadedSceneControlWidget.h"

ShadedSceneControlWidget::ShadedSceneControlWidget(QWidget *parent) :
    ParametersControlWidgetBase(parent),
    ui(new Ui::ShadedSceneControlWidget)
{
    ui->setupUi(this);
    QObject::connect(ui->edgeComboBox , SIGNAL(currentIndexChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(ui->faceComboBox , SIGNAL(currentIndexChanged(int)), this, SIGNAL(paramsChanged()));
    QObject::connect(ui->pointComboBox, SIGNAL(currentIndexChanged(int)), this, SIGNAL(paramsChanged()));
}

ShadedSceneControlWidget::~ShadedSceneControlWidget()
{
    delete ui;
}

ShadedSceneControlParameters *ShadedSceneControlWidget::createParameters() const
{
    ShadedSceneControlParameters *result = new ShadedSceneControlParameters();
    getParameters(*result);
    return result;
}

void ShadedSceneControlWidget::getParameters(ShadedSceneControlParameters &params) const
{
    params.edge.type  = ui->edgeComboBox->currentIndex()  == 0 ? ShaderPreset::NONE : ShaderPreset::PRESET;
    params.face.type  = ui->faceComboBox->currentIndex()  == 0 ? ShaderPreset::NONE : ShaderPreset::PRESET;
    params.point.type = ui->pointComboBox->currentIndex() == 0 ? ShaderPreset::NONE : ShaderPreset::PRESET;
}

void ShadedSceneControlWidget::setParameters(const ShadedSceneControlParameters &input)
{
    if ( input.edge.type == ShaderPreset::NONE) {
        ui->edgeComboBox->setCurrentIndex(0);
    } else {
        ui->edgeComboBox->setCurrentIndex(1);
    }

    if ( input.face.type == ShaderPreset::NONE) {
        ui->faceComboBox->setCurrentIndex(0);
    } else {
        ui->faceComboBox->setCurrentIndex(1);
    }

    if ( input.point.type == ShaderPreset::NONE) {
        ui->pointComboBox->setCurrentIndex(0);
    } else {
        ui->pointComboBox->setCurrentIndex(1);
    }
}

void ShadedSceneControlWidget::setParametersVirtual(void *input)
{
    // Modify widget parameters from outside
    ShadedSceneControlParameters *inputCasted = static_cast<ShadedSceneControlParameters *>(input);
    setParameters(*inputCasted);
}
