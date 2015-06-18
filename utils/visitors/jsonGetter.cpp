#include "jsonGetter.h"

#include <QtCore/QDebug>
#include <QtCore/QFile>
#include <QJsonDocument>

#include "jsonGetter.h"

JSONGetter::JSONGetter(const QString &fileName)
{
    mFileName = fileName;
    QFile file(mFileName);
    if (!file.open(QFile::ReadWrite)) {
        qDebug() << "Can't open file <" << mFileName << ">";
        return;
    }
    QDomDocument doc("document");
    doc.setContent(&file);

    qDebug() << doc.toString();
    file.close();

    mNodePath.push_back(doc);
}

template <>
void JSONGetter::visit<bool>(bool &boolField, bool defaultValue, const char *fieldName)
{

    QDomElement childElement = getChildByTag(fieldName);
    if (childElement.isNull())
    {
        boolField = defaultValue;
        return;
    }
    boolField = childElement.attribute("value", defaultValue ? "true" : "false") == "true";
}

template <>
void JSONGetter::visit<double>(double &doubleField, double defaultValue, const char *fieldName)
{
    QDomElement childElement = getChildByTag(fieldName);
    if (childElement.isNull())
    {
        doubleField = defaultValue;
        return;
    }
    doubleField = childElement.attribute("value", QString::number(defaultValue)).toDouble();
}

template <>
void JSONGetter::visit<float>(float &floatField, float defaultValue, const char *fieldName)
{
    QDomElement childElement = getChildByTag(fieldName);
    if (childElement.isNull())
    {
        floatField = defaultValue;
        return;
    }
    floatField = childElement.attribute("value", QString::number(defaultValue)).toFloat();
}

template <>
void JSONGetter::visit<int>(int &intField, int defaultValue, const char *fieldName)
{
    QDomElement childElement = getChildByTag(fieldName);
    if (childElement.isNull())
    {
        intField = defaultValue;
        return;
    }

    intField = childElement.attribute("value", QString::number(defaultValue)).toInt();
}

template <>
void JSONGetter::visit<std::string>(std::string &stringField, std::string defaultValue, const char *fieldName)
{
    QDomElement childElement = getChildByTag(fieldName);
    if (childElement.isNull())
    {
        stringField = defaultValue;
        return;
    }

    stringField = childElement.attribute("value", QString::fromStdString(defaultValue)).toUtf8().constData();
}

/* And new style visitor method */

template <>
void JSONGetter::visit<int, IntField>(int &intField, const IntField * fieldDescriptor)
{
    visit<int>(intField, fieldDescriptor->defaultValue, fieldDescriptor->name.name);
//	qDebug() << "JSONGetter::visit<int, IntField>(int &field, const IntField *fieldDescriptor) NOT YET SUPPORTED";
}

template <>
void JSONGetter::visit<double, DoubleField>(double &doubleField, const DoubleField * fieldDescriptor)
{
    visit<double>(doubleField, fieldDescriptor->defaultValue, fieldDescriptor->name.name);
//	qDebug() << "JSONGetter::visit<int, DoubleField>(double &field, const DoubleField *fieldDescriptor) NOT YET SUPPORTED";
}

template <>
void JSONGetter::visit<float, FloatField>(float &floatField, const FloatField * fieldDescriptor)
{
    visit<float>(floatField, fieldDescriptor->defaultValue, fieldDescriptor->name.name);
//	qDebug() << "JSONGetter::visit<float, FloatField>(float &field, const FloatField *fieldDescriptor) NOT YET SUPPORTED";
}

template <>
void JSONGetter::visit<bool, BoolField>(bool &boolField, const BoolField * fieldDescriptor)
{
    visit<bool>(boolField, fieldDescriptor->defaultValue, fieldDescriptor->name.name);
//	qDebug() << "JSONGetter::visit<int, BoolField>(bool &boolField, const BoolField *fieldDescriptor) NOT YET SUPPORTED";
}

template <>
void JSONGetter::visit<std::string, StringField>(std::string &stringField, const StringField * fieldDescriptor)
{
    visit<std::string>(stringField, fieldDescriptor->defaultValue, fieldDescriptor->name.name);
//	qDebug() << "JSONGetter::visit<int, StringField>(std::string &field, const StringField *fieldDescriptor) NOT YET SUPPORTED";
}

template <>
void JSONGetter::visit<void *, PointerField>(void * &/*field*/, const PointerField * /*fieldDescriptor*/)
{
    qDebug() << "JSONGetter::visit<int, PointerField>(void * &field, const PointerField * /*fieldDescriptor*/) NOT YET SUPPORTED";
}

template <>
void JSONGetter::visit<int, EnumField>(int &/*field*/, const EnumField * /*fieldDescriptor*/)
{
    qDebug() << "JSONGetter::visit<int, EnumField>(int &field, const EnumField *fieldDescriptor) NOT YET SUPPORTED";
}
