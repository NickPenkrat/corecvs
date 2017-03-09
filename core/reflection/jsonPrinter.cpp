#include "jsonPrinter.h"

namespace corecvs {

using std::string;
using std::endl;

const std::string JSONPrinter::PROLOGUE = "{";
const std::string JSONPrinter::EPILOGUE = "}";

const std::string JSONPrinter::OBJECT_OPEN  = "{";
const std::string JSONPrinter::OBJECT_CLOSE = "}";

const std::string JSONPrinter::ARRAY_OPEN  = "[";
const std::string JSONPrinter::ARRAY_CLOSE = "]";

const std::string JSONPrinter::FIELD_SEPARATOR = ",";
const std::string JSONPrinter::FIELD_VALUE_SEPARATOR = ":";

const std::string JSONPrinter::NAME_DECORATOR = "\"";

std::string JSONPrinter::decorateName(const BaseField *field)
{
    return JSONPrinter::NAME_DECORATOR + field->getSimpleName() + JSONPrinter::NAME_DECORATOR;
}

std::string JSONPrinter::decorateName(const char *field)
{
    return JSONPrinter::NAME_DECORATOR + field + JSONPrinter::NAME_DECORATOR;
}

template <>
void JSONPrinter::visit<int,    IntField>(int &field, const IntField *fieldDescriptor)
{
    if (stream == NULL) return;
    *stream << separate() << indent() << decorateName(fieldDescriptor) <<  FIELD_VALUE_SEPARATOR <<  field;
}

template <>
void JSONPrinter::visit<double, DoubleField>(double &field, const DoubleField *fieldDescriptor)
{
    if (stream == NULL) return;
    *stream << separate() << indent() << decorateName(fieldDescriptor) <<  FIELD_VALUE_SEPARATOR <<  field;
}

template <>
void JSONPrinter::visit<float,  FloatField>(float &field, const FloatField *fieldDescriptor)
{
    if (stream == NULL) return;
    *stream << separate() << indent() << decorateName(fieldDescriptor) <<  FIELD_VALUE_SEPARATOR <<  field;
}

template <>
void JSONPrinter::visit<bool,   BoolField>(bool &field, const BoolField *fieldDescriptor)
{
    if (stream == NULL) return;
    *stream << separate() << indent() << decorateName(fieldDescriptor) <<  FIELD_VALUE_SEPARATOR <<  field;
}

template <>
void JSONPrinter::visit<string, StringField>(std::string &field, const StringField *fieldDescriptor)
{
    if (stream == NULL) return;
    *stream << separate() << indent() << decorateName(fieldDescriptor) <<  FIELD_VALUE_SEPARATOR <<  NAME_DECORATOR << field << NAME_DECORATOR;
}

template <>
void JSONPrinter::visit<void *, PointerField>(void * &field, const PointerField *fieldDescriptor)
{
    if (stream == NULL) return;
    *stream << separate() << indent() << decorateName(fieldDescriptor) <<  FIELD_VALUE_SEPARATOR <<  field;
}

template <>
void JSONPrinter::visit<int, EnumField>(int &field, const EnumField *fieldDescriptor)
{
    if (stream == NULL) return;
    *stream << separate() << indent() << decorateName(fieldDescriptor) <<  FIELD_VALUE_SEPARATOR <<  field;
}


/* Arrays block */

template <>
void JSONPrinter::visit<double, DoubleVectorField>(std::vector<double> &field, const DoubleVectorField *fieldDescriptor)
{
    if (stream == NULL) return;
    *stream << separate() << indent() << decorateName(fieldDescriptor) << FIELD_VALUE_SEPARATOR << " " << ARRAY_OPEN;
    for (size_t i = 0; i < field.size(); i++ )
    {
        *stream << ( i == 0 ? " " : ", ") << field[i] ;
    }
    *stream << ARRAY_CLOSE;
}

/* Old style visitor */

template <>
void JSONPrinter::visit<bool>(bool &boolField, bool /*defaultValue*/, const char *fieldName)
{
    if (stream == NULL) return;
    *stream << separate() << indent() << decorateName(fieldName) <<  FIELD_VALUE_SEPARATOR << NAME_DECORATOR << (boolField ? "true" : "false") << NAME_DECORATOR;
}

template <>
void JSONPrinter::visit<std::string>(std::string &stringField, std::string /*defaultValue*/, const char *fieldName)
{
    if (stream == NULL) return;
    *stream << separate() << indent() << decorateName(fieldName) <<  FIELD_VALUE_SEPARATOR << NAME_DECORATOR << stringField << NAME_DECORATOR;
}




} //namespace corecvs
