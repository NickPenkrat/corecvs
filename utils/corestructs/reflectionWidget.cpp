#include <QGridLayout>
#include <QLabel>
#include <QSpinBox>
#include <QComboBox>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QTextEdit>
#include <QPushButton>


#include "reflection.h"
#include "reflectionWidget.h"
#include "vectorWidget.h"
#include "exponentialSlider.h"

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
        QWidget *widget = NULL;

        switch (field->type) {
        case BaseField::TYPE_INT:
        {
            const IntField *iField = static_cast<const IntField *>(field);
            QSpinBox *spinBox = new QSpinBox(this);

            if (iField->hasAdditionalValues)
            {
                spinBox->setMinimum(iField->min);
                spinBox->setMaximum(iField->max);

                if (iField->step != 0) {
                    spinBox->setSingleStep(iField->step);
                }
                qDebug("ReflectionWidget::Setting limits(%d %d %d)", iField->min, iField->max, iField->step);
            }
            spinBox->setValue(iField->defaultValue);
            if (iField->suffixHint != NULL)
                spinBox->setSuffix(iField->suffixHint);
            if (iField->prefixHint != NULL)
                spinBox->setPrefix(iField->prefixHint);

            layout->addWidget(spinBox, i, 1, 1, 1);
            connect(spinBox, SIGNAL(valueChanged(int)), this, SIGNAL(paramsChanged()));
            widget = spinBox;
            break;
        }
        case BaseField::TYPE_DOUBLE:
        {
            const DoubleField *dField = static_cast<const DoubleField *>(field);

            if (dField->widgetHint != BaseField::SLIDER)
            {
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
                widget = spinBox;
            } else {
                 ExponentialSlider *expBox = new ExponentialSlider(this);
                 if (dField->hasAdditionalValues)
                 {
                     expBox->setMaxZoom(dField->max);
                 }
                 expBox->setValue(dField->defaultValue);
                 layout->addWidget(expBox, i, 1, 1, 1);
                 connect(expBox, SIGNAL(valueChanged(double)), this, SIGNAL(paramsChanged()));
                 widget = expBox;
            }
            break;
        }
        case BaseField::TYPE_STRING:
        {
            const StringField *sField = static_cast<const StringField *>(field);
            QTextEdit *textBox = new QTextEdit(this);
            textBox->setText(QString::fromStdString(sField->defaultValue));
            layout->addWidget(textBox, i, 1, 1, 1);
            connect(textBox, SIGNAL(textChanged()), this, SIGNAL(paramsChanged()));
            widget = textBox;
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
            widget = checkBox;
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
                if (option->presentationHint != NULL && strlen(option->presentationHint) != 0)
                {
                    const QString iconName = QString(option->presentationHint).split("@")[0];
                    comboBox->addItem(QIcon(iconName), option->name.name);

                } else {
                    comboBox->addItem(option->name.name);
                }
            }
            comboBox->setCurrentIndex(eField->defaultValue);
            layout->addWidget(comboBox, i, 1, 1, 1);
            connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SIGNAL(paramsChanged()));
            widget = comboBox;
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
            connect(vectorWidget, SIGNAL(valueChanged()), this, SIGNAL(paramsChanged()));
            widget = vectorWidget;
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
                widget = refWidget;
            }
            break;
        }

        /* Composite field */
        case BaseField::TYPE_POINTER:
        {
            const PointerField *pField = static_cast<const PointerField *>(field);
            QString targetName = pField->name.name;

            QPushButton *buttonWidget = new QPushButton(this);
            layout->addWidget(buttonWidget, i, 1, 1, 1);
            buttonWidget->setEnabled(false);
            buttonWidget->setText(targetName);
            //connect(buttonWidget, SIGNAL(paramsChanged()), this, SIGNAL(paramsChanged()));
            widget = buttonWidget;
            break;
        }

        /* Well something is not supported */
        default:
            QLabel *label = new QLabel(this);
            label->setText(QString("NOT SUPPORTED"));
            layout->addWidget(label, i, 1, 1, 1);
            break;
        }

        fieldToWidget.push_back(widget);
    }

    QSpacerItem *spacer = new QSpacerItem(0, 20, QSizePolicy::Expanding, QSizePolicy::Expanding);
    layout->addItem(spacer, (int)reflection->fields.size(), 0, 1, 2);

    if (reflection->isActionBlock()) {
        QPushButton *executeButton = new QPushButton(this);
        executeButton->setIcon(QIcon(":/new/prefix1/lightning.png"));
        executeButton->setText("Execute");
        layout->addWidget(executeButton, layout->rowCount(), 1, 1, 1);

    }

    setLayout(layout);
}

bool ReflectionWidget::getParameters(void *param) const
{
    DynamicObject obj(reflection, param);

    for (int i = 0; i < (int)reflection->fields.size(); i++)
    {
        const BaseField *field = reflection->fields[i];
        qDebug() << "Processing field:" <<  field->getSimpleName();

        switch (field->type) {
            case BaseField::TYPE_INT:
            {
                // const IntField *iField = static_cast<const IntField *>(field);
                QSpinBox *spinBox = static_cast<QSpinBox *>(fieldToWidget[i]);
                *obj.getField<int>(i) = spinBox->value();
                break;
            }
            case BaseField::TYPE_DOUBLE:
            {
                const DoubleField *dField = static_cast<const DoubleField *>(field);
                if (dField->widgetHint != BaseField::SLIDER)
                {
                    QDoubleSpinBox *spinBox = static_cast<QDoubleSpinBox *>(fieldToWidget[i]);
                    *obj.getField<double>(i) = spinBox->value();
                } else {
                    ExponentialSlider *expBox = static_cast<ExponentialSlider *>(fieldToWidget[i]);
                    *obj.getField<double>(i) = expBox->value();
                }
                break;
            }
            case BaseField::TYPE_STRING:
            {
                // const StringField *sField = static_cast<const StringField *>(field);
                QTextEdit *textBox = static_cast<QTextEdit *>(fieldToWidget[i]);
                *obj.getField<std::string>(i) = textBox->toPlainText().toStdString();
                break;
            }
                break;
            case BaseField::TYPE_BOOL:
            {
                QCheckBox *checkBox = static_cast<QCheckBox *>(fieldToWidget[i]);
                *obj.getField<bool>(i) = checkBox->isChecked();
                break;
            }
            case BaseField::TYPE_ENUM:
            {
                // const EnumField *eField = static_cast<const EnumField *>(field);
                QComboBox *comboBox = static_cast<QComboBox *>(fieldToWidget[i]);
                *obj.getField<int>(i) = comboBox->currentIndex();
                break;
            }
            /* Two Vector fields*/
            /*case BaseField::TYPE_INT | BaseField::TYPE_VECTOR_BIT:
            {
                const IntField *iField = static_cast<const IntField *>(field);
                break;
            }*/
            case BaseField::TYPE_DOUBLE | BaseField::TYPE_VECTOR_BIT:
            {
                // const DoubleVectorField *dField = static_cast<const DoubleVectorField *>(field);
                DoubleVectorWidget *vectorWidget = static_cast<DoubleVectorWidget *>(fieldToWidget[i]);
                *obj.getField<vector<double>>(i) = vectorWidget->value();
                break;
            }

            /* Composite field */
            case BaseField::TYPE_COMPOSITE:
            {
                const CompositeField *cField = static_cast<const CompositeField *>(field);
                // const Reflection *subReflection = cField->reflection;
                ReflectionWidget *refWidget = static_cast<ReflectionWidget *>(fieldToWidget[i]);

                if (refWidget != NULL) {
                    refWidget->getParameters(obj.getField<void *>(i));
                } else {
                    qDebug() << "There is no widget for" << cField->getSimpleName();
                }
                break;
            }

            /* Well something is not supported */
            default:
                break;
        }
    }

    return true;
}

bool ReflectionWidget::setParameters(void *param) const
{
    DynamicObject obj(reflection, param);

    for (size_t i = 0; i < reflection->fields.size(); i++)
    {
        const BaseField *field = reflection->fields[i];
        qDebug() << "Processing field:" <<  field->getSimpleName();

        switch (field->type) {
            case BaseField::TYPE_INT:
            {
                QSpinBox *spinBox = static_cast<QSpinBox *>(fieldToWidget[i]);
                spinBox->setValue(*obj.getField<int>(i));
                break;
            }
            case BaseField::TYPE_DOUBLE:
            {
                const DoubleField *dField = static_cast<const DoubleField *>(field);
                if (dField->widgetHint != BaseField::SLIDER)
                {
                    QDoubleSpinBox *spinBox = static_cast<QDoubleSpinBox *>(fieldToWidget[i]);
                    spinBox->setValue(*obj.getField<double>(i));
                } else {
                    ExponentialSlider *expBox = static_cast<ExponentialSlider *>(fieldToWidget[i]);
                    expBox->setValue(*obj.getField<double>(i));
                }
                break;
            }
            case BaseField::TYPE_STRING:
            {
                QTextEdit *textBox = static_cast<QTextEdit *>(fieldToWidget[i]);
                textBox->setText(QString::fromStdString(*obj.getField<std::string>(i)));
                break;
            }
                break;
            case BaseField::TYPE_BOOL:
            {
                QCheckBox *checkBox = static_cast<QCheckBox *>(fieldToWidget[i]);
                checkBox->setChecked(*obj.getField<bool>(i));
                break;
            }
            case BaseField::TYPE_ENUM:
            {
                QComboBox *comboBox = static_cast<QComboBox *>(fieldToWidget[i]);
                comboBox->setCurrentIndex(*obj.getField<int>(i));
                break;
            }
            /* Two Vector fields*/
            /*case BaseField::TYPE_INT | BaseField::TYPE_VECTOR_BIT:
            {
                const IntField *iField = static_cast<const IntField *>(field);
                break;
            }*/
            case BaseField::TYPE_DOUBLE | BaseField::TYPE_VECTOR_BIT:
            {
                DoubleVectorWidget *vectorWidget = static_cast<DoubleVectorWidget *>(fieldToWidget[i]);

                break;
            }

            /* Composite field */
            case BaseField::TYPE_COMPOSITE:
            {
                const CompositeField *cField = static_cast<const CompositeField *>(field);
                ReflectionWidget *refWidget = static_cast<ReflectionWidget *>(fieldToWidget[i]);
                if (refWidget != NULL) {
                    refWidget->setParameters(obj.getField<void *>(i));
                } else {
                     qDebug() << "There is no widget for" << cField->getSimpleName();
                }
                break;
            }

            /* Well something is not supported */
            default:
                break;
        }
    }

    return true;
}

