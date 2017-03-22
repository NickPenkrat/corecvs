#ifndef JSON_PRINTER_H
#define JSON_PRINTER_H


#include <stdint.h>
#include <iostream>
#include <sstream>

#include "reflection.h"

namespace corecvs {

using std::string;
using std::ostream;
using std::cout;


/**
   Please note, json is finallised on destruction of the object.
**/
class JSONPrinter
{
public:
    bool isSaver () { return false;}
    bool isLoader() { return false;}


public:
    std::ostream *stream;
    int indentation;
    int dIndent;
    bool isFirst;

    static const std::string PROLOGUE;
    static const std::string EPILOGUE;

    static const std::string OBJECT_OPEN;
    static const std::string OBJECT_CLOSE;

    static const std::string ARRAY_OPEN;
    static const std::string ARRAY_CLOSE;

    static const std::string FIELD_SEPARATOR;
    static const std::string FIELD_VALUE_SEPARATOR;
    static const std::string NAME_DECORATOR;



    explicit  JSONPrinter(ostream *_stream) :
        stream(_stream),
        indentation(0),
        dIndent(1)
    {
        prologue();
    }

    explicit  JSONPrinter(ostream &_stream = cout) :
        stream(&_stream),
        indentation(0),
        dIndent(1)
    {
        prologue();
    }

    explicit  JSONPrinter(int indent, int dindent, ostream &_stream = cout) :
        stream(&_stream),
        indentation(indent),
        dIndent(dindent)
    {
        prologue();
    }

    ~JSONPrinter()
    {
        epilogue();
    }

    std::string indent() {
        return std::string(indentation, ' ');
    }

    /*json prologue*/
    void prologue() {
        if (stream == NULL) return;
        (*stream) << PROLOGUE;
        isFirst = true;
    }

    void epilogue() {
        if (stream == NULL) return;
        (*stream) << std::endl << EPILOGUE;
    }

    std::string separate() {
        std::string result;
        if (!isFirst && (stream != NULL))
        {
            result = ",\n";
        } else {
            result = "\n";
        }
        isFirst = false;
        return result;
    }

    /**
     * Generic Array support with new style reflection
     **/
    template <typename inputType, typename reflectionType>
    void visit(std::vector<inputType> &fields, const reflectionType * /*fieldDescriptor*/)
    {
        indentation += dIndent;
        for (int i = 0; i < fields.size(); i++)
        {
            fields[i].accept(*this);
        }
        indentation -= dIndent;
    }

    /**
     * Generic Array support with old style
     **/
    template <typename innerType>
    void visit(std::vector<innerType> &field, const char* arrayName)
    {
        indentation += dIndent;
        for (size_t i = 0; i < field.size(); i++)
        {
            std::ostringstream ss;
            ss << NAME_DECORATOR << arrayName << NAME_DECORATOR << "[" <<  i << "]";
            visit<innerType>(field[i], innerType(), ss.str().c_str());
        }
        indentation -= dIndent;
    }

    /**
     * Complex objects support with new style reflection
     **/
template <typename inputType, typename reflectionType>
    void visit(inputType &field, const reflectionType * fieldDescriptor)
    {
        if (stream != NULL) {
            *stream << separate() << indent() << decorateName(fieldDescriptor) << FIELD_VALUE_SEPARATOR << OBJECT_OPEN;
        }
        indentation += dIndent;
        field.accept(*this);
        indentation -= dIndent;

        if (stream != NULL) {
            *stream << std::endl << indent() << OBJECT_CLOSE;
        }
    }


    /**
     * Complex objects support with old style calls
     **/
template <class Type>
    void visit(Type &field, const char * fieldName)
    {
        if (stream != NULL) {
            *stream << separate() << indent() << decorateName(fieldName) << FIELD_VALUE_SEPARATOR << OBJECT_OPEN;
        }
        indentation += dIndent;
        isFirst = true;
        field.accept(*this);
        indentation -= dIndent;

        if (stream != NULL) {
            *stream << std::endl << indent() << OBJECT_CLOSE;
        }
    }

    template <typename Type,
              typename std::enable_if<!(std::is_enum<Type>::value || (std::is_arithmetic<Type>::value && !std::is_same<bool, Type>::value)), int>::type foo = 0>
    void visit(Type &field, Type /*defaultValue*/, const char * fieldName)
    {
        visit<Type>(field, fieldName);
    }

    template <typename type, typename std::enable_if<std::is_arithmetic<type>::value && !std::is_same<bool, type>::value, int>::type foo = 0>
    void visit(type &field, type, const char *fieldName)
    {
        if (stream == NULL) return;
        *stream << separate() << indent() << decorateName(fieldName) << FIELD_VALUE_SEPARATOR << field;
    }

    template <typename type, typename std::enable_if<std::is_enum<type>::value, int>::type foo = 0>
    void visit(type &field, type defaultValue, const char *fieldName)
    {
        using U = typename std::underlying_type<type>::type;
        U u = static_cast<U>(field);
        visit(u, static_cast<U>(defaultValue), fieldName);
        field = static_cast<type>(u);
    }

private:

    static std::string decorateName(const BaseField *field);
    static std::string decorateName(const char *field);
};


template <>
void JSONPrinter::visit<int,    IntField>(int &field, const IntField *fieldDescriptor);

template <>
void JSONPrinter::visit<uint64_t, UInt64Field>(uint64_t &field, const UInt64Field *fieldDescriptor);

template <>
void JSONPrinter::visit<double, DoubleField>(double &field, const DoubleField *fieldDescriptor);

template <>
void JSONPrinter::visit<float,  FloatField>(float &field, const FloatField *fieldDescriptor);

template <>
void JSONPrinter::visit<bool,   BoolField>(bool &field, const BoolField *fieldDescriptor);

template <>
void JSONPrinter::visit<string, StringField>(string &field, const StringField *fieldDescriptor);

template <>
void JSONPrinter::visit<void *, PointerField>(void * &field, const PointerField *fieldDescriptor);

template <>
void JSONPrinter::visit<int,    EnumField>(int &field, const EnumField *fieldDescriptor);

template <>
void JSONPrinter::visit<std::string, StringField>(std::string &field, const StringField *fieldDescriptor);

/* Arrays */

template <>
void JSONPrinter::visit<double, DoubleVectorField>(std::vector<double> &field, const DoubleVectorField *fieldDescriptor);

/**
 * Old Style
 *
 * this methods can be made universal, but are separated to make it a bit more controllable
 **/
template <>
void JSONPrinter::visit<uint64_t>(uint64_t &intField, uint64_t defaultValue, const char *fieldName);

template <>
void JSONPrinter::visit<bool>(bool &boolField, bool defaultValue, const char *fieldName);

template <>
void JSONPrinter::visit<std::string>(std::string &stringField, std::string defaultValue, const char *fieldName);


} //namespace corecvs

#endif // JSON_PRINTER_H
