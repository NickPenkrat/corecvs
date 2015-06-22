#ifndef JSONGETTER_H
#define JSONGETTER_H

#include <QtCore/QString>
#include <QJsonDocument>
#include <QJsonObject>
#include "reflection.h"

using corecvs::IntField;
using corecvs::DoubleField;
using corecvs::FloatField;
using corecvs::BoolField;
using corecvs::StringField;
using corecvs::PointerField;
using corecvs::EnumField;

using namespace corecvs;

class JSONGetter
{
public:
    /**
     *  Create a getter object that will use data from a file with a specified name.
     *
     **/
    JSONGetter(QString const & fileName);

    /**
     *  Create a getter object that will use data from a given XML
     **/
    JSONGetter(QJsonObject &document) :
        mDocument(document)
    {
        mNodePath.push_back(mDocument);
    }

    /**
     *  Visitor method that will traverse xml and object tree and fill object with data form xml
     *
     *
     **/
    template <class Type>
        void visit(Type &field, Type defaultValue, const char *fieldName)
    {
        pushChild(fieldName);
            field.accept(*this);
        popChild();
    }

    template <class Type>
        void visit(Type &field, const char *fieldName)
        {
            pushChild(fieldName);
                field.accept(*this);
            popChild();
        }

    template <typename inputType, typename reflectionType>
        void visit(inputType &field, const reflectionType * fieldDescriptor)
    {
        pushChild(fieldDescriptor->getSimpleName());
           field.accept(*this);
        popChild();
    }


    void pushChild(const char *childName)
    {
        QJsonObject mainNode = mNodePath.back();
        QJsonObject value    = mainNode.value(childName).toObject();
        mNodePath.push_back(value);
    }

    void popChild()
    {
        mNodePath.pop_back();
    }

private:
    std::vector<QJsonObject> mNodePath;
    QString     mFileName;
    QJsonObject mDocument;
};

template <>
void JSONGetter::visit<int>(int &intField, int defaultValue, const char * /*fieldName*/);

template <>
void JSONGetter::visit<double>(double &doubleField, double defaultValue, const char * /*fieldName*/);

template <>
void JSONGetter::visit<float>(float &floatField, float defaultValue, const char * /*fieldName*/);

template <>
void JSONGetter::visit<bool>(bool &boolField, bool defaultValue, const char * /*fieldName*/);

/* New style visitor */

template <>
void JSONGetter::visit<int, IntField>(int &field, const IntField *fieldDescriptor);

template <>
void JSONGetter::visit<double, DoubleField>(double &field, const DoubleField *fieldDescriptor);

template <>
void JSONGetter::visit<float, FloatField>(float &field, const FloatField *fieldDescriptor);

template <>
void JSONGetter::visit<bool, BoolField>(bool &field, const BoolField *fieldDescriptor);

template <>
void JSONGetter::visit<std::string, StringField>(std::string &field, const StringField *fieldDescriptor);

template <>
void JSONGetter::visit<void *, PointerField>(void * &field, const PointerField *fieldDescriptor);

template <>
void JSONGetter::visit<int, EnumField>(int &field, const EnumField *fieldDescriptor);

#endif // JSONGETTER_H
