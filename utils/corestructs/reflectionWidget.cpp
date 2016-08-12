#include <QGridLayout>
#include <QLabel>
#include <QSpinBox>
#include <QComboBox>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QTextEdit>

#include "reflection.h"
#include "reflectionWidget.h"
#include "vectorWidget.h"

using namespace corecvs;

ReflectionWidget::ReflectionWidget(const Reflection *reflection) :
    reflection(reflection)
{

    resize(396, 356);

    setWindowTitle(reflection->name.name);

    QGridLayout *layout = new QGridLayout(this);
    layout->setSpacing(3);
    layout->setContentsMargins(3, 3, 3, 3);

    for (size_t i = 0; i < reflection->fields.size(); i++)
    {
        const BaseField *field = reflection->fields[i];
        qDebug() << "Processing field:" <<  field->getSimpleName();

        QLabel *label = new QLabel(this);
        label->setText(QString(field->getSimpleName()));
        label->setToolTip(QString(field->name.decription));
        layout->addWidget(label, i, 0, 1, 1);

        switch (field->type) {
        case BaseField::TYPE_INT:
        {
            const IntField *iField = static_cast<const IntField *>(field);
            QSpinBox *spinBox = new QSpinBox(this);

            if (iField->hasAdditionalValues)
            {
                spinBox->setMinimum(iField->min);
                spinBox->setMaximum(iField->max);
                spinBox->setSingleStep(iField->step);
            }
            spinBox->setValue(iField->defaultValue);
            if (iField->suffixHint != NULL)
                spinBox->setSuffix(iField->suffixHint);
            if (iField->prefixHint != NULL)
                spinBox->setPrefix(iField->prefixHint);

            layout->addWidget(spinBox, i, 1, 1, 1);
            connect(spinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
            break;
        }
        case BaseField::TYPE_DOUBLE:
        {
            const DoubleField *dField = static_cast<const DoubleField *>(field);
            QDoubleSpinBox *spinBox = new QDoubleSpinBox(this);

            if (dField->hasAdditionalValues)
            {
                spinBox->setMinimum(dField->min);
                spinBox->setMaximum(dField->max);
                spinBox->setSingleStep(dField->step);
                // spinBox->setDecimals(); /*Not supported so far*/
            }
            spinBox->setDecimals(dField->precision);
            if (dField->suffixHint != NULL)
                spinBox->setSuffix(dField->suffixHint);
            if (dField->prefixHint != NULL)
                spinBox->setPrefix(dField->prefixHint);

            spinBox->setValue(dField->defaultValue);
            layout->addWidget(spinBox, i, 1, 1, 1);
            connect(spinBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
            break;
        }
        case BaseField::TYPE_STRING:
        {
            const StringField *sField = static_cast<const StringField *>(field);
            QTextEdit *textBox = new QTextEdit(this);
            textBox->setText(QString::fromStdString(sField->defaultValue));
            layout->addWidget(textBox, i, 1, 1, 1);
            connect(textBox, SIGNAL(textChanged()), this, SIGNAL(paramsChanged()));
            break;
        }
            break;
        case BaseField::TYPE_BOOL:
        {
            const BoolField *bField = static_cast<const BoolField *>(field);
            QCheckBox *checkBox = new QCheckBox(this);
            checkBox->setChecked(bField->defaultValue);
            layout->addWidget(checkBox, i, 1, 1, 1);
            connect(checkBox, SIGNAL(toggled(bool)), this, SIGNAL(paramsChanged()));
            break;
        }
        case BaseField::TYPE_ENUM:
        {
            const EnumField *eField = static_cast<const EnumField *>(field);
            const EnumReflection *enumOptions = eField->enumReflection;

            QComboBox *comboBox = new QComboBox(this);

            for(size_t enumCount = 0; enumCount < enumOptions->options.size(); enumCount++)
            {
                const EnumOption *option = enumOptions->options[enumCount];
                comboBox->addItem(option->name.name);
            }
            comboBox->setCurrentIndex(eField->defaultValue);
            layout->addWidget(comboBox, i, 1, 1, 1);
            connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SIGNAL(paramsChanged()));
            break;
        }
        /* Two Vector fields*/
        /*case BaseField::TYPE_INT | BaseField::TYPE_VECTOR_BIT:
        {
            const IntField *iField = static_cast<const IntField *>(field);
            QSpinBox *spinBox = new QSpinBox(this);

            if (iField->hasAdditionalValues)
            {
                spinBox->setMinimum(iField->min);
                spinBox->setMaximum(iField->max);
                spinBox->setSingleStep(iField->step);
            }
            spinBox->setValue(iField->defaultValue);
            layout->addWidget(spinBox, i, 1, 1, 1);
            connect(spinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
            break;
        }*/
        case BaseField::TYPE_DOUBLE | BaseField::TYPE_VECTOR_BIT:
        {
            const DoubleVectorField *dField = static_cast<const DoubleVectorField *>(field);
            DoubleVectorWidget *vectorWidget = new DoubleVectorWidget(this);

            if (dField->hasAdditionalValues)
            {
                vectorWidget->setMinimum(dField->min);
                vectorWidget->setMaximum(dField->max);
                // vectorWidget->setSingleStep(dField->step);
                // spinBox->setDecimals(); /*Not supported so far*/
            }
            vectorWidget->setValue(dField->defaultValue);
            layout->addWidget(vectorWidget, i, 1, 1, 1);
            connect(vectorWidget, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
            break;
        }

        /* Composite field */
        case BaseField::TYPE_COMPOSITE:
        {
            const CompositeField *cField = static_cast<const CompositeField *>(field);
            const Reflection *subReflection = cField->reflection;
            if (subReflection != NULL) {
                ReflectionWidget *refWidget = new ReflectionWidget(subReflection);
                layout->addWidget(refWidget, i, 1, 1, 1);
                connect(refWidget, SIGNAL(paramsChanged()), this, SIGNAL(paramsChanged()));
            }


            break;
        }

        /* Well something is not supported */
        default:
            QLabel *label = new QLabel(this);
            label->setText(QString("NOT SUPPORTED"));
            layout->addWidget(label, i, 1, 1, 1);
            break;
        }

    }

    QSpacerItem *spacer = new QSpacerItem(0, 20, QSizePolicy::Expanding, QSizePolicy::Expanding);
    layout->addItem(spacer, reflection->fields.size(), 0, 1, 2);

    setLayout(layout);
}

BaseReflectionStatic *ReflectionWidget::createParametersVirtual() const
{

}
