#include "core/reflection/binaryVisitor.h"
#include "core/utils/utils.h"

namespace corecvs {

std::string BinaryVisitor::decorateName(const BaseField *field)
{
    return BinaryVisitor::NAME_DECORATOR + field->getSimpleName() + BinaryVisitor::NAME_DECORATOR;
}

std::string BinaryVisitor::decorateName(const char *field)
{
    return BinaryVisitor::NAME_DECORATOR + field + BinaryVisitor::NAME_DECORATOR;
}

std::string BinaryVisitor::escapeString(const std::string &str)
{
    return HelperUtils::escapeString(str,
    {{'"', '"'},
     {'\\', '\\'},
     {'\b','b'},
     {'\f', 'f'},
     {'\r', 'r'},
     {'\n', 'n'},
     {'\t', 't'}},"\\");
}

template <>
void BinaryVisitor::visit<int,    IntField>(int &field, const IntField *fieldDescriptor)
{
    if (stream == NULL) return;
    *stream << separate() << indent() << decorateName(fieldDescriptor) <<  FIELD_VALUE_SEPARATOR <<  field;
}


template <>
void BinaryVisitor::visit<double, DoubleField>(double &field, const DoubleField *fieldDescriptor)
{
    if (stream == NULL) return;
    *stream << separate() << indent() << decorateName(fieldDescriptor) <<  FIELD_VALUE_SEPARATOR <<  field;
}

template <>
void BinaryVisitor::visit<float,  FloatField>(float &field, const FloatField *fieldDescriptor)
{
    if (stream == NULL) return;
    *stream << separate() << indent() << decorateName(fieldDescriptor) <<  FIELD_VALUE_SEPARATOR <<  field;
}

template <>
void BinaryVisitor::visit<uint64_t, UInt64Field>(uint64_t &field, const UInt64Field *fieldDescriptor)
{
    if (stream == NULL) return;
    *stream << separate() << indent() << decorateName(fieldDescriptor) <<  FIELD_VALUE_SEPARATOR <<  NAME_DECORATOR << field << "u64" << NAME_DECORATOR;
}


template <>
void BinaryVisitor::visit<bool,   BoolField>(bool &field, const BoolField *fieldDescriptor)
{
    if (stream == NULL) return;
    *stream << separate() << indent() << decorateName(fieldDescriptor) <<  FIELD_VALUE_SEPARATOR <<  (field ? "true" : "false");
}

template <>
void BinaryVisitor::visit<string, StringField>(std::string &field, const StringField *fieldDescriptor)
{
    if (stream == NULL) return;
    *stream << separate() << indent() << decorateName(fieldDescriptor) <<  FIELD_VALUE_SEPARATOR <<  NAME_DECORATOR << escapeString(field) << NAME_DECORATOR;
}

template <>
void BinaryVisitor::visit<std::wstring, WStringField>(std::wstring &field, const WStringField *fieldDescriptor)
{
    if (stream == NULL) return;
    *stream << separate() << indent() << decorateName(fieldDescriptor) <<  FIELD_VALUE_SEPARATOR <<  NAME_DECORATOR << /*escapeString(field)*/ "Unsupported" << NAME_DECORATOR;
}

template <>
void BinaryVisitor::visit<void *, PointerField>(void * &field, const PointerField *fieldDescriptor)
{
    if (stream == NULL) return;
    *stream << separate() << indent() << decorateName(fieldDescriptor) <<  FIELD_VALUE_SEPARATOR <<  field;
}

template <>
void BinaryVisitor::visit<int, EnumField>(int &field, const EnumField *fieldDescriptor)
{
    if (stream == NULL) return;
    *stream << separate() << indent() << decorateName(fieldDescriptor) <<  FIELD_VALUE_SEPARATOR <<  field;
}


/* Arrays block */

template <>
void BinaryVisitor::visit<double, DoubleVectorField>(std::vector<double> &field, const DoubleVectorField *fieldDescriptor)
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
void BinaryVisitor::visit<uint64_t>(uint64_t &intField, uint64_t /*defaultValue*/, const char *fieldName)
{
    if (stream == NULL) return;
    *stream << separate() << indent() << decorateName(fieldName) <<  FIELD_VALUE_SEPARATOR <<  NAME_DECORATOR << intField << "u64" << NAME_DECORATOR;;
}

template <>
void BinaryVisitor::visit<bool>(bool &boolField, bool /*defaultValue*/, const char *fieldName)
{
    if (stream == NULL) return;
    *stream << separate() << indent() << decorateName(fieldName) <<  FIELD_VALUE_SEPARATOR << NAME_DECORATOR << (boolField ? "true" : "false") << NAME_DECORATOR;
}

template <>
void BinaryVisitor::visit<std::string>(std::string &stringField, std::string /*defaultValue*/, const char *fieldName)
{
    if (stream == NULL) return;
    *stream << separate() << indent() << decorateName(fieldName) <<  FIELD_VALUE_SEPARATOR << NAME_DECORATOR << escapeString(stringField) << NAME_DECORATOR;
}

template <>
void BinaryVisitor::visit<std::wstring>(std::wstring &stringField, std::wstring /*defaultValue*/, const char *fieldName)
{
    if (stream == NULL) return;
    *stream << separate() << indent() << decorateName(fieldName) <<  FIELD_VALUE_SEPARATOR << NAME_DECORATOR << /*escapeString(stringField)*/ "Unsupported" << NAME_DECORATOR;
}




} //namespace corecvs
