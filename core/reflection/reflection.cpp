#include "reflection.h"

namespace corecvs {

/* Well... we expect some compiler magic to work and init it before the first use */
std::unique_ptr<ReflectionDirectory> ReflectionDirectoryHolder::reflectionDirectory;

ReflectionDirectory *ReflectionDirectoryHolder::getReflectionDirectory()
{
    if (!reflectionDirectory)
        reflectionDirectory.reset(new ReflectionDirectory());

    /* We lose uniquness here. That is not a big problem. Quite obvious that ReflectionDirectory has global ownership */
    return reflectionDirectory.get();
}


bool DynamicObject::simulateConstructor()
{
    if (reflection == NULL || rawObject == NULL)
        return false;

    for (int count = 0; count < reflection->fieldNumber(); count++)
    {
        const BaseField *field = reflection->fields[count];
        switch (field->type) {
        case BaseField::TYPE_BOOL:
        {
            const BoolField *tfield = static_cast<const BoolField *>(field);
            *getField<bool>(count) = tfield->defaultValue;
            break;
        }
        case BaseField::TYPE_INT:
        {
            const IntField *tfield = static_cast<const IntField *>(field);
            *getField<int>(count) = tfield->defaultValue;
            break;
        }
        case BaseField::TYPE_DOUBLE:
        {
            const DoubleField *tfield = static_cast<const DoubleField *>(field);
            *getField<double>(count) = tfield->defaultValue;
            break;
        }
        case BaseField::TYPE_STRING:
        {
            const StringField *tfield = static_cast<const StringField *>(field);
            *getField<std::string>(count) = tfield->defaultValue;
            break;
        }
        case BaseField::TYPE_WSTRING:
        {
            const WStringField *tfield = static_cast<const WStringField *>(field);
            *getField<std::wstring>(count) = tfield->defaultValue;
            break;
        }
        case (BaseField::FieldType)(BaseField::TYPE_VECTOR_BIT | BaseField::TYPE_DOUBLE) :
        {
            const DoubleVectorField *tfield = static_cast<const DoubleVectorField *>(field);
            *getField<vector<double> >(count) = tfield->defaultValue;
            break;
        }
        case BaseField::TYPE_COMPOSITE:
        {
            const CompositeField *tfield = static_cast<const CompositeField *>(field);
            if (tfield->reflection == NULL)
            {
                SYNC_PRINT(("DynamicObject::simulateConstructor(%s): composite field (%d : %s) has empty reflection",
                            reflection->name.name, count, tfield->name.name
                            ));
                break;
            }
            DynamicObject obj(tfield->reflection, getField<void *>(count));
            obj.simulateConstructor();
            break;
        }

        default:
            break;
        }
    }

    return true;
}

bool BaseField::isInputPin() const {
    const char *prefix = "in";
    return strncmp(name.name, prefix, strlen(prefix)) == 0;
}

bool BaseField::isOuputPin() const {
    const char *prefix = "out";
    return strncmp(name.name, prefix, strlen(prefix)) == 0;
}

bool Reflection::isActionBlock() const
{
    for (int fieldId = 0; fieldId < fieldNumber(); fieldId++)
    {
        const BaseField *field = fields[fieldId];
        if (field->isInputPin()) {
            return true;
        }
        if (field->isOuputPin()) {
            return false;
        }
    }
    return false;
}




} // namespace corecvs

