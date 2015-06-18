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
        mNodePath.push_back(mDocument);
    }

    virtual ~JSONSetter();

template <class Type>
    void visit(Type &field, Type defaultValue, const char *fieldName)
    {
        pushChild(fieldName);
           field.accept(*this);
        popChild();
    }

template <class inputType>
    void visit(inputType &field, const char *fieldName)
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
        mNodePath.push_back(QJsonObject());
        mChildName = childName;
        SYNC_PRINT(("push %s. Stack size %d\n", childName, mNodePath.size()));
    }

    void popChild()
    {
        QJsonObject mainNode = mNodePath.back();
        mNodePath.pop_back();
        mNodePath.back().insert(mChildName, mainNode);
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


#endif // JSONSETTER_H
