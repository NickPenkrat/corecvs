#ifndef CORETOSCRIPT_H
#define CORETOSCRIPT_H

#include "reflection.h"

#include "axisAlignedBoxParameters.h"

#include <QMetaType>
#include <QScriptValue>
#include <QScriptContext>

#ifndef WORK_IN_PROGRESS

/* We have to do it for all types */
Q_DECLARE_METATYPE(AxisAlignedBoxParameters);


class ReflectionToScript
{
public:
    corecvs::Reflection *mReflection = NULL;

    ReflectionToScript(corecvs::Reflection *reflection = NULL) :
        mReflection(reflection)
    {}

template<class ExposeType>
    static QScriptValue createMyStruct(QScriptContext *, QScriptEngine *engine)
    {
        ExposeType s;
        return QScriptValue(); //engine->toScriptValue(s);
    }

template<class ExposeType>
    static QScriptValue toScriptValue(QScriptEngine *engine, const ExposeType &s)
    {
          corecvs::Reflection *reflection = &ExposeType::reflection;

          DynamicObject input(&s);

          QScriptValue obj = engine->newObject();
          if (reflection == NULL)
          {
              return obj;
          }

          for (size_t i = 0; i < reflection->fields.size(); i++)
          {
              const BaseField *field = reflection->fields[i];
              QString name = field->name.name;
              switch (field->type) {
              case BaseField::TYPE_INT:
              {
                  //const IntField *iField = static_cast<const IntField *>(field);
                  obj.setProperty(name, *(input.template getField<int>(i)));
                  break;
              }
              case BaseField::TYPE_DOUBLE:
              {
                  //const DoubleField *dField = static_cast<const DoubleField *>(field);
                  obj.setProperty(name, *input.template getField<double>(i));
                  break;
              }
              case BaseField::TYPE_STRING:
              {
                  //const StringField *sField = static_cast<const StringField *>(field);
                  obj.setProperty(name, QString::fromStdString(*input.template getField<std::string>(i)));
                  break;
              }
              case BaseField::TYPE_BOOL:
              {
                  //const BoolField *bField = static_cast<const BoolField *>(field);
                  obj.setProperty(name, *input.template getField<bool>(i));
                  break;
              }
              case BaseField::TYPE_ENUM:
              {
                  const EnumField *eField = static_cast<const EnumField *>(field);
                  const EnumReflection *enumOptions = eField->enumReflection;
                  /* NOT SUPPORTED */
                  break;
              }
              case BaseField::TYPE_DOUBLE | BaseField::TYPE_VECTOR_BIT:
              {
                  const DoubleVectorField *dField = static_cast<const DoubleVectorField *>(field);
                  // DoubleVectorWidget *vectorWidget = new DoubleVectorWidget(this);
                  /* NOT SUPPORTED */
                  break;
              }

              /* Composite field */
              case BaseField::TYPE_COMPOSITE:
              {
                  const CompositeField *cField = static_cast<const CompositeField *>(field);
                  const Reflection *subReflection = cField->reflection;
                   /* NOT SUPPORTED */
                  break;
              }

              /* Composite field */
              case BaseField::TYPE_POINTER:
              {
                  /* NOT SUPPORTED */
                  break;
              }
          }
      }

      return obj;
    }

template<class ExposeType>
    static void fromScriptValue(const QScriptValue &obj, ExposeType &s)
    {
        corecvs::Reflection *reflection = &ExposeType::reflection;

        DynamicObject input(&s);

        if (reflection == NULL)
        {
            return;
        }

        for (size_t i = 0; i < reflection->fields.size(); i++)
        {
            const BaseField *field = reflection->fields[i];
            QString name = field->name.name;

            QScriptValue value = obj.property(name);

            switch (field->type) {
            case BaseField::TYPE_INT:
            {
                *(input.template getField<int>(i)) = value.toInt32();
                break;
            }
            case BaseField::TYPE_DOUBLE:
            {
                *input.template getField<double>(i) = value.toNumber();
                break;
            }
            case BaseField::TYPE_STRING:
            {
                *input.template getField<std::string>(i) = value.toString().toStdString();
                break;
            }
            case BaseField::TYPE_BOOL:
            {
                *input.template getField<bool>(i) = value.toBool();
                break;
            }
            case BaseField::TYPE_ENUM:
            {
                const EnumField *eField = static_cast<const EnumField *>(field);
                const EnumReflection *enumOptions = eField->enumReflection;
                /* NOT SUPPORTED */
                break;
            }
            case BaseField::TYPE_DOUBLE | BaseField::TYPE_VECTOR_BIT:
            {
                const DoubleVectorField *dField = static_cast<const DoubleVectorField *>(field);
                //DoubleVectorWidget *vectorWidget = new DoubleVectorWidget(this);
                /* NOT SUPPORTED */
                break;
            }

            /* Composite field */
            case BaseField::TYPE_COMPOSITE:
            {
                const CompositeField *cField = static_cast<const CompositeField *>(field);
                //const Reflection *subReflection = cField->reflection;
                 /* NOT SUPPORTED */
                break;
            }

            /* Composite field */
            case BaseField::TYPE_POINTER:
            {
                /* NOT SUPPORTED */
                break;
            }
            }
        }
    }
};

#endif
#endif // CORETOSCRIPT_H

