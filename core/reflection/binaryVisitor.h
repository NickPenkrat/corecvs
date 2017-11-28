#ifndef BINARY_VISITOR_H
#define BINARY_VISITOR_H


#include <stdint.h>
#include <iostream>
#include <sstream>
#include <fstream>

#include "core/reflection/reflection.h"
#include "core/utils/log.h"

namespace corecvs {

using std::string;
using std::ostream;
using std::cout;


/**
   \attention NB Please note!!

   This visitor is not stroing data in "key->value" form.
   It only stores values, so it assumes fields would be requested in the same order they have been writen

   There would be no checks or warnings so far. You would just experience fast yet painful death
**/
class BinaryVisitor
{
public:
    bool isSaver () { return true;}
    bool isLoader() { return false;}


public:
    std::ostream *stream  = NULL;
    bool fileOwned = false;


    explicit  BinaryVisitor(std::ostream *ostream):
        stream(ostream)
    {
        prologue();
    }

    explicit  BinaryVisitor(const string &filepath)
        : stream(new std::ofstream(filepath.c_str(), std::ofstream::out)),
          fileOwned(true)
    {
        if (!(*stream))
        {
            L_ERROR_P("Couldn't open for writting the file <%s>", filepath.c_str());
            delete_safe(stream);
            fileOwned = false;
            return;
        }
        L_INFO_P("saving to <%s>", filepath.c_str());
        prologue();
    }

    ~BinaryVisitor()
    {
       if (fileOwned) {
           epilogue();
           delete_safe(stream);
           fileOwned = false;
       }
    }
    /*file prologue*/
    void prologue() {
        const char *greeting="MJKPACK";
        if (stream == NULL) return;
        stream->write(greeting, strlen(greeting));
    }

    void epilogue() {
        const char *bye="EOF";
        if (stream == NULL) return;
        stream->write(bye, strlen(bye));
    }


    /* */
    template <typename innerType>
    void visit(std::vector<std::vector<innerType>> &fields, const char *arrayName)
    {

    }

    /**
     * Generic Array support with new style reflection
     **/
    template <typename inputType, typename reflectionType>
    void visit(std::vector<inputType> &fields, const reflectionType * /*fieldDescriptor*/)
    {
        for (int i = 0; i < fields.size(); i++)
        {
            fields[i].accept(*this);
        }        
    }

    /**
     * Generic Array support with old style
     **/
    template <typename innerType>
    void visit(std::vector<innerType> &field, const char* arrayName)
    {

    }

    /**
     * Complex objects support with new style reflection
     **/
template <typename inputType, typename reflectionType>
    void visit(inputType &field, const reflectionType * fieldDescriptor)
    {       
        field.accept(*this);       
    }


    /**
     * Complex objects support with old style calls
     **/
template <class Type>
    void visit(Type &field, const char * fieldName)
    {
        field.accept(*this);
    }

template <typename Type>
    void visit(Type &field, Type /*defaultValue*/, const char * fieldName)
    {
        visit<Type>(field, fieldName);
    }

};


template <>
void BinaryVisitor::visit<int,    IntField>(int &field, const IntField *fieldDescriptor);

template <>
void BinaryVisitor::visit<uint64_t, UInt64Field>(uint64_t &field, const UInt64Field *fieldDescriptor);

template <>
void BinaryVisitor::visit<double, DoubleField>(double &field, const DoubleField *fieldDescriptor);

template <>
void BinaryVisitor::visit<float,  FloatField>(float &field, const FloatField *fieldDescriptor);

template <>
void BinaryVisitor::visit<bool,   BoolField>(bool &field, const BoolField *fieldDescriptor);

template <>
void BinaryVisitor::visit<void *, PointerField>(void * &field, const PointerField *fieldDescriptor);

template <>
void BinaryVisitor::visit<int,    EnumField>(int &field, const EnumField *fieldDescriptor);

template <>
void BinaryVisitor::visit<std::string, StringField>(std::string &field, const StringField *fieldDescriptor);

template <>
void BinaryVisitor::visit<std::wstring, WStringField>(std::wstring &field, const WStringField *fieldDescriptor);


/* Arrays */

template <>
void BinaryVisitor::visit<double, DoubleVectorField>(std::vector<double> &field, const DoubleVectorField *fieldDescriptor);

/**
 * Old Style
 *
 * this methods can be made universal, but are separated to make it a bit more controllable
 **/
template <>
void BinaryVisitor::visit<uint64_t>(uint64_t &intField, uint64_t defaultValue, const char *fieldName);

template <>
void BinaryVisitor::visit<bool>(bool &boolField, bool defaultValue, const char *fieldName);

template <>
void BinaryVisitor::visit<std::string>(std::string &stringField, std::string defaultValue, const char *fieldName);

template <>
void BinaryVisitor::visit<std::wstring>(std::wstring &stringField, std::wstring defaultValue, const char *fieldName);


} //namespace corecvs

#endif // BINARY_VISITOR_H
