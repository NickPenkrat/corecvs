#include "core/reflection/binaryVisitor.h"
#include "core/utils/utils.h"

namespace corecvs {


template <>
void BinaryVisitor::visit<int,    IntField>(int &field, const IntField *fieldDescriptor)
{
    if (stream == NULL) return;
    stream->write((char *) &field, sizeof(field));
}


template <>
void BinaryVisitor::visit<double, DoubleField>(double &field, const DoubleField *fieldDescriptor)
{
    if (stream == NULL) return;
    stream->write((char *) &field, sizeof(field));
}

template <>
void BinaryVisitor::visit<float,  FloatField>(float &field, const FloatField *fieldDescriptor)
{
    if (stream == NULL) return;
    stream->write((char *) &field, sizeof(field));
}

template <>
void BinaryVisitor::visit<uint64_t, UInt64Field>(uint64_t &field, const UInt64Field *fieldDescriptor)
{
    if (stream == NULL) return;
    stream->write((char *) &field, sizeof(field));
}


template <>
void BinaryVisitor::visit<bool,   BoolField>(bool &field, const BoolField *fieldDescriptor)
{
    if (stream == NULL) return;
    stream->write((char *) &field, sizeof(field));
}

template <>
void BinaryVisitor::visit<string, StringField>(std::string &field, const StringField *fieldDescriptor)
{
    if (stream == NULL) return;
    SYNC_PRINT(("%s : NYI\n", __FUNCTION__));
    //stream->write((char *) &field, sizeof(field));
}

template <>
void BinaryVisitor::visit<std::wstring, WStringField>(std::wstring &field, const WStringField *fieldDescriptor)
{
    if (stream == NULL) return;
    SYNC_PRINT(("%s : NYI\n", __FUNCTION__));
}

template <>
void BinaryVisitor::visit<void *, PointerField>(void * &field, const PointerField *fieldDescriptor)
{
    if (stream == NULL) return;
    SYNC_PRINT(("%s : NYI\n", __FUNCTION__));
}

template <>
void BinaryVisitor::visit<int, EnumField>(int &field, const EnumField *fieldDescriptor)
{
    if (stream == NULL) return;
    stream->write((char *) &field, sizeof(field));
}


/* Arrays block */

template <>
void BinaryVisitor::visit<double, DoubleVectorField>(std::vector<double> &field, const DoubleVectorField *fieldDescriptor)
{
    if (stream == NULL) return;
    SYNC_PRINT(("%s : NYI\n", __FUNCTION__));
}

/* Old style visitor */

template <>
void BinaryVisitor::visit<uint64_t>(uint64_t &intField, uint64_t /*defaultValue*/, const char *fieldName)
{
    if (stream == NULL) return;
    stream->write((char *) &intField, sizeof(intField));
}

template <>
void BinaryVisitor::visit<bool>(bool &boolField, bool /*defaultValue*/, const char *fieldName)
{
    if (stream == NULL) return;
    stream->write((char *) &boolField, sizeof(boolField));
}

template <>
void BinaryVisitor::visit<std::string>(std::string &stringField, std::string /*defaultValue*/, const char *fieldName)
{
    if (stream == NULL) return;
    SYNC_PRINT(("%s : NYI\n", __FUNCTION__));
}

template <>
void BinaryVisitor::visit<std::wstring>(std::wstring &stringField, std::wstring /*defaultValue*/, const char *fieldName)
{
    if (stream == NULL) return;
    SYNC_PRINT(("%s : NYI\n", __FUNCTION__));
}




} //namespace corecvs
