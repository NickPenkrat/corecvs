#include "featurePointControlWidget.h"
#include "ui_featurePointControlWidget.h"

FeaturePointControlWidget::FeaturePointControlWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::FeaturePointControlWidget)
{
    ui->setupUi(this);

    QObject::connect(ui->nameLineEdit, SIGNAL(textChanged(QString)), this, SIGNAL(paramsChanged()));

    QObject::connect(ui->xSpinBox,     SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(ui->ySpinBox,     SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
    QObject::connect(ui->zSpinBox,     SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));

    QObject::connect(ui->hasPositionCheckBox,     SIGNAL(toggled(bool)), this, SIGNAL(paramsChanged()));

    QObject::connect(ui->color,     SIGNAL(paramsChanged(bool)), this, SIGNAL(paramsChanged()));


}



FeaturePointControlWidget::~FeaturePointControlWidget()
{
    delete ui;
}

void FeaturePointControlWidget::getParameters(SceneFeaturePoint &params) const
{
    params.name = ui->nameLineEdit->text().toStdString();

    params.position.x() = ui->xSpinBox->value();
    params.position.y() = ui->ySpinBox->value();
    params.position.z() = ui->zSpinBox->value();

    params.hasKnownPosition = ui->hasPositionCheckBox->isChecked();

    params.color = ui->color->getColor();

}

void FeaturePointControlWidget::setParameters(const SceneFeaturePoint &input)
{
    // Block signals to send them all at once
    bool wasBlocked = blockSignals(true);
    ui->nameLineEdit->setText(QString::fromStdString(input.name));

    ui->xSpinBox->setValue(input.position.x());
    ui->ySpinBox->setValue(input.position.y());
    ui->zSpinBox->setValue(input.position.z());

    ui->hasPositionCheckBox->setChecked(input.hasKnownPosition);

    ui->color->setRGBColor(input.color);


    blockSignals(wasBlocked);
    emit paramsChanged();
}

void FeaturePointControlWidget::setParametersVirtual(void *input)
{
    // Modify widget parameters from outside
    SceneFeaturePoint *inputCasted = static_cast<SceneFeaturePoint *>(input);
    setParameters(*inputCasted);
}

