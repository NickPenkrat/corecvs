#ifndef PRINTER_VISITOR_OLD_H_
#define PRINTER_VISITOR_OLD_H_

#include <iostream>

#include "reflection.h"

namespace corecvs {

using std::string;
using std::ostream;
using std::cout;

class PrinterVisitor
{
public:
    std::ostream *stream;
    int indentation;

    explicit  PrinterVisitor(ostream *_stream) :
        stream(_stream),
        indentation(0)
    {}

    explicit  PrinterVisitor(ostream &_stream = cout) :
        stream(&_stream),
        indentation(0)
    {}

    std::string ident() {
        return std::string(" ", indentation);
    }

/* Generic Array support */
    template <typename inputType, typename reflectionType>
    void visit(std::vector<inputType> &fields, const reflectionType * /*fieldDescriptor*/)
    {
        indentation++;
        for (int i = 0; i < fields.size(); i++)
        {
            fields[i].accept(*this);
        }
        indentation--;
    }

template <typename inputType, typename reflectionType>
    void visit(inputType &field, const reflectionType * fieldDescriptor)
    {
        if (stream != NULL) {
            *stream << ident() << fieldDescriptor->getSimpleName() << ":" << std::endl;
        }
        indentation++;
        field.accept(*this);
        indentation--;
    }

template <class Type>
    void visit(Type &field, Type /*defaultValue*/, const char * fieldName)
	{
        if (stream != NULL) {
            *stream << ident() << fieldName << ":" << std::endl;
        }
        indentation++;
		field.accept(*this);
        indentation--;
	}

};


template <>
void PrinterVisitor::visit<int,    IntField>(int &field, const IntField *fieldDescriptor);

template <>
void PrinterVisitor::visit<double, DoubleField>(double &field, const DoubleField *fieldDescriptor);

template <>
void PrinterVisitor::visit<float,  FloatField>(float &field, const FloatField *fieldDescriptor);

template <>
void PrinterVisitor::visit<bool,   BoolField>(bool &field, const BoolField *fieldDescriptor);

template <>
void PrinterVisitor::visit<string, StringField>(string &field, const StringField *fieldDescriptor);

template <>
void PrinterVisitor::visit<void *, PointerField>(void * &field, const PointerField *fieldDescriptor);

template <>
void PrinterVisitor::visit<int,    EnumField>(int &field, const EnumField *fieldDescriptor);

template <>
void PrinterVisitor::visit<std::string, StringField>(std::string &field, const StringField *fieldDescriptor);

/* Arrays */

template <>
void PrinterVisitor::visit<double, DoubleVectorField>(std::vector<double> &field, const DoubleVectorField *fieldDescriptor);

/* Old style */

template <>
void PrinterVisitor::visit<int>(int &doubleField, int defaultValue, const char *fieldName);

template <>
void PrinterVisitor::visit<double>(double &doubleField, double defaultValue, const char *fieldName);

template <>
void PrinterVisitor::visit<float>(float &floatField, float defaultValue, const char *fieldName);

template <>
void PrinterVisitor::visit<bool>(bool &boolField, bool defaultValue, const char *fieldName);

template <>
void PrinterVisitor::visit<std::string>(std::string &stringField, std::string defaultValue, const char *fieldName);


} //namespace corecvs
#endif // PRINTER_VISITOR_OLD_H_
