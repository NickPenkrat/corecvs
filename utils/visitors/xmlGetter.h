#pragma once

#include <QtCore/QString>
#include <QtXml/QDomElement>
#include "reflection.h"

using corecvs::IntField;
using corecvs::DoubleField;
using corecvs::FloatField;
using corecvs::BoolField;
using corecvs::StringField;
using corecvs::PointerField;
using corecvs::EnumField;

#include "basePathVisitor.h"

using namespace corecvs;

class XmlGetter
{
public:
    XmlGetter(QString const & fileName);

    XmlGetter(QDomElement &documentElement) :
        mDocumentElement(documentElement)
    {
        mElementPath.push_back(mDocumentElement);
    }


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

private:

    QDomElement getChildByTag(const char *name);

    void pushChild(const char *childName)
    {
        mElementPath.push_back(getChildByTag(childName));
    }

    void popChild()
    {
        mElementPath.pop_back();
    }

    std::vector<QDomElement> mElementPath;

    QString mFileName;
    QDomElement mDocumentElement;
};

template <>
void XmlGetter::visit<int>(int &intField, int defaultValue, const char * /*fieldName*/);

template <>
void XmlGetter::visit<double>(double &doubleField, double defaultValue, const char * /*fieldName*/);

template <>
void XmlGetter::visit<float>(float &floatField, float defaultValue, const char * /*fieldName*/);

template <>
void XmlGetter::visit<bool>(bool &boolField, bool defaultValue, const char * /*fieldName*/);

/* New style visitor */

template <>
void XmlGetter::visit<int, IntField>(int &field, const IntField *fieldDescriptor);

template <>
void XmlGetter::visit<double, DoubleField>(double &field, const DoubleField *fieldDescriptor);

template <>
void XmlGetter::visit<float, FloatField>(float &field, const FloatField *fieldDescriptor);

template <>
void XmlGetter::visit<bool, BoolField>(bool &field, const BoolField *fieldDescriptor);

template <>
void XmlGetter::visit<std::string, StringField>(std::string &field, const StringField *fieldDescriptor);

template <>
void XmlGetter::visit<void *, PointerField>(void * &field, const PointerField *fieldDescriptor);

template <>
void XmlGetter::visit<int, EnumField>(int &field, const EnumField *fieldDescriptor);
