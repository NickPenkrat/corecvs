#include "core/reflection/advanced/advancedBinaryReader.h"
#include "core/utils/utils.h"

namespace corecvs {

template<typename Type>
void  AdvancedBinaryReader::loadField(Type &field, const std::string &name)
{
    if (stream == NULL) return;

    auto it = dicts.back().find(name);
    if (it == dicts.back().end())
    {
        std::cout << "Name not found:" << name << std::endl;
        return;
    }

    stream->seekg(it->second);

    uint32_t size;
    stream->read((char *) &size, sizeof(size));
    if (size != sizeof(Type)) {
        std::cout << "Internal format error" << std::endl;
    }

    cout << "Reading from" << stream->tellg() << std::endl;
    stream->read((char *) &field, sizeof(field));
}


template <>
void AdvancedBinaryReader::visit<int,    IntField>(int &field, const IntField *fieldDescriptor)
{
    if (stream == NULL) return;
    loadField<int>(field, fieldDescriptor->name.name);
    SYNC_PRINT(("AdvancedBinaryReader::visit<int,IntField>(): read %d\n", field));
}

template <>
void AdvancedBinaryReader::visit<double, DoubleField>(double &field, const DoubleField *fieldDescriptor)
{
    if (stream == NULL) return;
    loadField<double>(field, fieldDescriptor->name.name);
    SYNC_PRINT(("AdvancedBinaryReader::visit<double, DoubleField>():read %lf\n", field));
}

template <>
void AdvancedBinaryReader::visit<float,  FloatField>(float &field, const FloatField *fieldDescriptor)
{
    if (stream == NULL) return;
    loadField<float>(field, fieldDescriptor->name.name);
    SYNC_PRINT(("AdvancedBinaryReader::visit<float, FloatField>():read %f\n", field));

}

template <>
void AdvancedBinaryReader::visit<uint64_t, UInt64Field>(uint64_t &field, const UInt64Field *fieldDescriptor)
{
    if (stream == NULL) return;
    loadField<uint64_t>(field, fieldDescriptor->name.name);
    stream->read((char *) &field, sizeof(field));
}


template <>
void AdvancedBinaryReader::visit<bool,   BoolField>(bool &field, const BoolField *fieldDescriptor)
{
    if (stream == NULL) return;
    SYNC_PRINT(("%s : NYI\n", __FUNCTION__));
}

template <>
void AdvancedBinaryReader::visit<string, StringField>(std::string &field, const StringField *fieldDescriptor)
{
    if (stream == NULL) return;
    SYNC_PRINT(("%s : NYI\n", __FUNCTION__));
    //stream->read((char *) &field, sizeof(field));
}

template <>
void AdvancedBinaryReader::visit<std::wstring, WStringField>(std::wstring &field, const WStringField *fieldDescriptor)
{
    if (stream == NULL) return;
    SYNC_PRINT(("%s : NYI\n", __FUNCTION__));
}

template <>
void AdvancedBinaryReader::visit<void *, PointerField>(void * &field, const PointerField *fieldDescriptor)
{
    if (stream == NULL) return;
    SYNC_PRINT(("%s : NYI\n", __FUNCTION__));
}

template <>
void AdvancedBinaryReader::visit<int, EnumField>(int &field, const EnumField *fieldDescriptor)
{
    if (stream == NULL) return;
    loadField<int>(field, fieldDescriptor->name.name);
}


/* Arrays block */

template <>
void AdvancedBinaryReader::visit<double, DoubleVectorField>(std::vector<double> &field, const DoubleVectorField *fieldDescriptor)
{
    if (stream == NULL) return;

    SYNC_PRINT(("%s : NYI\n", __FUNCTION__));
}

/* Old style visitor */

template <>
void AdvancedBinaryReader::visit<uint64_t>(uint64_t &field, uint64_t /*defaultValue*/, const char *fieldName)
{
    if (stream == NULL) return;
    loadField<uint64_t>(field, fieldName);
}

template <>
void AdvancedBinaryReader::visit<bool>(bool &field, bool /*defaultValue*/, const char *fieldName)
{
    if (stream == NULL) return;
    loadField<bool>(field, fieldName);
}

template <>
void AdvancedBinaryReader::visit<int>(int &field, int defaultValue, const char *fieldName)
{
    if (stream == NULL) return;
    loadField<int>(field, fieldName);
    SYNC_PRINT(("AdvancedBinaryReader::visit<int>(): read %d\n", field));
}


template <>
void AdvancedBinaryReader::visit<double>(double &field, double defaultValue, const char *fieldName)
{
    if (stream == NULL) return;
    loadField<double>(field, fieldName);
    SYNC_PRINT(("AdvancedBinaryReader::visit<double>(): read %lf\n", field));
}

template <>
void AdvancedBinaryReader::visit<std::string>(std::string &stringField, std::string /*defaultValue*/, const char *fieldName)
{
    if (stream == NULL) return;
    uint32_t length = 0;
    stream->read((char *)&length, sizeof(length));
    char* data = new char[length + 1];
    stream->read((char *)data, length);
    data[length] = 0;
    stringField = data;
    SYNC_PRINT(("AdvancedBinaryReader::visit<std::string>():read %s\n", stringField.c_str()));
}

template <>
void AdvancedBinaryReader::visit<std::wstring>(std::wstring &stringField, std::wstring /*defaultValue*/, const char *fieldName)
{
    if (stream == NULL) return;
    SYNC_PRINT(("%s : NYI\n", __FUNCTION__));
}




} //namespace corecvs
