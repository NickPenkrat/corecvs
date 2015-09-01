#ifndef JSONSETTER_H
#define JSONSETTER_H

#include <QtCore/QtCore>
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

using corecvs::DoubleVectorField;

using namespace corecvs;

class JSONSetter
{
public:
    /**
     *  Create a setter object that will store data to a file with a specified name.
     *
     **/
    JSONSetter(QString const & filename);

    /**
     *  Create a setter object that will store data to a given XML
     **/
    JSONSetter(QJsonObject &document)
    {
        mNodePath.push_back(document);
    }

    virtual ~JSONSetter();

template <class Type>
    void visit(Type &field, Type defaultValue, const char *fieldName)
    {
        pushChild(fieldName);
           field.accept(*this);
        popChild(fieldName);
    }

template <class inputType>
    void visit(inputType &field, const char *fieldName)
    {
        pushChild(fieldName);
            field.accept(*this);
        popChild(fieldName);
    }

template <typename inputType, typename reflectionType>
    void visit(inputType &field, const reflectionType * fieldDescriptor)
    {
        pushChild(fieldDescriptor->getSimpleName());
           field.accept(*this);
        popChild(fieldDescriptor->getSimpleName());
    }

/* Generic Array support */
    template <typename inputType>
    void visit(std::vector<inputType> &fields, const char * arrayName)
    {
        QJsonArray array;
        int before = mNodePath.size();
        for (int i = 0; i < fields.size(); i++)
        {
            mNodePath.push_back(QJsonObject());
            fields[i].accept(*this);
            array.append(mNodePath.back());
            mNodePath.pop_back();
        }

        mNodePath.back().insert(arrayName, array);
    }

    template <typename inputType, typename reflectionType>
    void visit(std::vector<inputType> &fields, const reflectionType * /*fieldDescriptor*/)
    {
        for (int i = 0; i < fields.size(); i++)
        {
            fields[i].accept(*this);
        }
    }

    void pushChild(const char *childName)
    {
        mNodePath.push_back(QJsonObject());
     //   mChildName = childName;
        SYNC_PRINT(("push %s. Stack size %lu\n", childName, mNodePath.size()));
    }

    void popChild(const char *childName)
    {
        QJsonObject mainNode = mNodePath.back();
        mNodePath.pop_back();
        mNodePath.back().insert(childName, mainNode);
    }

private:
    std::vector<QJsonObject> mNodePath;
    QString     mFileName;
    QString     mChildName;
};

template <>
void JSONSetter::visit<int>(int &intField, int defaultValue, const char *fieldName);

template <>
void JSONSetter::visit<double>(double &doubleField, double defaultValue, const char *fieldName);

template <>
void JSONSetter::visit<float>(float &floatField, float defaultValue, const char *fieldName);

template <>
void JSONSetter::visit<bool>(bool &boolField, bool defaultValue, const char *fieldName);

template <>
void JSONSetter::visit<std::string>(std::string &stringField, std::string defaultValue, const char *fieldName);


/* New style visitor */

template <>
void JSONSetter::visit<int, IntField>(int &field, const IntField *fieldDescriptor);

template <>
void JSONSetter::visit<double, DoubleField>(double &field, const DoubleField *fieldDescriptor);

template <>
void JSONSetter::visit<float, FloatField>(float &field, const FloatField *fieldDescriptor);

template <>
void JSONSetter::visit<bool, BoolField>(bool &field, const BoolField *fieldDescriptor);

template <>
void JSONSetter::visit<std::string, StringField>(std::string &field, const StringField *fieldDescriptor);

template <>
void JSONSetter::visit<void *, PointerField>(void * &field, const PointerField *fieldDescriptor);

template <>
void JSONSetter::visit<int, EnumField>(int &field, const EnumField *fieldDescriptor);


/* Arrays */
template <>
void JSONSetter::visit<double, DoubleVectorField>(std::vector<double> &field, const DoubleVectorField *fieldDescriptor);

#endif // JSONSETTER_H
