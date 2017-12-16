/**
 * \file PODGenerator.cpp
 * \brief Add Comment Here
 *
 * \date Jan 2, 2012
 * \author alexander
 */

#include <algorithm>

#include <QDir>
#include <QtCore/QDebug>

#include "pdoGenerator.h"


using namespace std;

PDOGenerator::PDOGenerator(const Reflection *_clazz)
    : BaseGenerator(_clazz)
{
}

void PDOGenerator::enterFieldContext(int i)
{
    field = clazz->fields[i];
    name = field->name.name;
    fieldPlaceholder=QString("field%1").arg(i);
    type = field->type;
    cppName = QString("m") + toCamelCase(name, true);
    getterName = toCamelCase(name, false);
    setterName = QString("set") + toCamelCase(name, true);

    cppType = getCppTypeForType(field);
    fieldRefType = getFieldRefTypeForType(type);
    descr = field->name.decription;
    comment = field->name.comment;

    defaultValue = getDefaultValue(field);

    fieldEnumId = toEnumName(name) + "_ID";

    simplifiedType = "";
    if (type == BaseField::TYPE_POINTER)
    {
        simplifiedType = "(void * &)";
    }
    if (type == BaseField::TYPE_ENUM)
    {
        simplifiedType = "(int &)";
    }
    boxName = toCamelCase(name, false) + getWidgetSuffixForType(type);
    if (type == BaseField::TYPE_COMPOSITE)
    {
        QString typeName = static_cast<const CompositeField *>(field)->typeName;
        boxFileName = toCamelCase(typeName, false) + getWidgetSuffixForType(type);
    }
    if (type == BaseField::TYPE_COMPOSITE_ARRAY)
    {
        QString typeName = static_cast<const CompositeArrayField *>(field)->typeName;
        boxFileName = toCamelCase(typeName, false) + getWidgetSuffixForType(type);
    }

    boxSignal = getSignalForType(type);

    prefix = "";
    suffix = "";

    if (type == BaseField::TYPE_ENUM)
    {
        prefix = "static_cast<"+cppType+">(";
        suffix = ")";
    }
}

void PDOGenerator::generatePDOEnumSubH(const EnumReflection *eref)
{
    QString result;
  //  const EnumReflection *eref = efield->enumReflection;

    QString enumName = toCamelCase(QString(eref->name.name), true);
    QString enumComment = eref->name.comment;
    QString enumDescr   = eref->name.decription;

//    qDebug() << "Generating enum " << enumName;

    QString fileName    = toCamelCase(eref->name.name) + ".h";
    QString enumCapitalName = toEnumName(eref->name.name);
    QString guardDefine = enumCapitalName + "_H_";

    out.close();
    out.open(QString(getGenerateDir() + QDir::separator() + fileName).toLatin1(), ios::out);

    result+=
    "#ifndef "+guardDefine+"\n"
    "#define "+guardDefine+"\n"
    "/**\n"
    " * \\file "+fileName+"\n"
    " * \\attention This file is automatically generated and should not be in general modified manually\n"
    " *\n"
    " * \\date MMM DD, 20YY\n"
    " * \\author autoGenerator\n"
    " */\n"
    "\n"
    "/**\n"
    " * Helper namespace to hide "+enumName+" enum from global context.\n"
    " */\n"
    "\n"
    "namespace "+enumName+" {\n"
    "\n"
    "/** \n"
    " * \\brief "+enumDescr+" \n"
    " * "+enumComment+" \n"
    " */\n"
    "enum "+enumName+" {\n";

    for (int j = 0; j < eref->optionsNumber(); j ++)
    {
        const EnumOption *option = eref->options[j];
        QString optionName = toEnumName(QString(option->name.name));
        QString optionComment = option->name.comment;
        QString optionDescr   = option->name.decription;
        QString optionID   = QString::number(option->id);

    result+=
    "    /** \n"
    "     * \\brief "+optionDescr+" \n"
    "     * "+optionComment+" \n"
    "     */\n"
    "    " + optionName + " = " + optionID + ",\n";
    }

    result+=
    "    /** \n"
    "     * \\brief Last virtual option to run cycles to\n"
    "     */\n"
    "    "+enumCapitalName+"_LAST\n"
    "};\n"
    "\n"
    "\n"
    "static inline const char *getName(const "+ enumName + " &value)\n"
    "{\n"
    "    switch (value) \n"
    "    {\n";

    for (int j = 0; j < eref->optionsNumber(); j ++)
    {
        const EnumOption *option = eref->options[j];
        QString optionName = toEnumName(QString(option->name.name));
//        QString optionComment = option->name.comment;
//        QString optionDescr   = option->name.decription;
//        QString optionID   = QString::number(option->id);

    result+=
    "     case " + optionName + " : return \"" + optionName + "\"; break ;\n";
    }

    result+=
    "     default : return \"Not in range\"; break ;\n"
    "     \n"
    "    }\n"
    "    return \"Not in range\";\n"
    "}\n"
    "\n"
    "} //namespace "+enumName+"\n"
    "\n"
    "#endif  //"+guardDefine+"\n";

    out << result.toLatin1().constData();;
}


void PDOGenerator::generatePDOH()
{
    QString result;
    int fieldNumber = clazz->fields.size();
    int embeddedNumber = clazz->embeds.size();

    QString fileName    = toCamelCase(clazz->name.name) + ".h";
    QString classCapitalName = toEnumName(clazz->name.name);
    QString guardDefine = classCapitalName + "_H_";
    QString className   = toCamelCase(clazz->name.name, true);
    QString classComment = clazz->name.comment;
    QString classDescr   = clazz->name.decription;

    out.close();
    out.open(QString(getGenerateDir() + QDir::separator() + fileName).toLatin1(), ios::out);

    result +=
    "#ifndef "+guardDefine+"\n"
    "#define "+guardDefine+"\n"
    "/**\n"
    " * \\file "+fileName+"\n"
    " * \\attention This file is automatically generated and should not be in general modified manually\n"
    " *\n"
    " * \\date MMM DD, 20YY\n"
    " * \\author autoGenerator\n"
    " */\n"
    "\n"
    "#include \"core/reflection/reflection.h\"\n"
    "#include \"core/reflection/defaultSetter.h\"\n"
    "#include \"core/reflection/printerVisitor.h\"\n"
    "\n"
    "/*\n"
    " *  Embed includes.\n"
    " */\n";

    /*We don't bother vector is fast enough */
    vector<const Reflection *> references;
    for (int i = 0; i < embeddedNumber; i++ )
    {
        const Reflection *referent = clazz->embeds.at(i)->subclass;
        if (std::find(references.begin(), references.end(), referent) != references.end())
            continue;
    result +=
    "#include \"" + toCamelCase(referent->name.name) + ".h\"\n";
        references.push_back(referent);
    }

    result +=
    "/*\n"
    " *  Additional includes for Composite Types.\n"
    " */\n";

    for (int i = 0; i < fieldNumber; i++ )
    {
        enterFieldContext(i);
        const Reflection *referent = NULL;
        if (field->type == BaseField::TYPE_COMPOSITE)
        {
            const CompositeField *cfield = static_cast<const CompositeField *>(field);
            referent = cfield->reflection;
        }
        if (field->type == BaseField::TYPE_COMPOSITE_ARRAY)
        {
            const CompositeArrayField *cafield = static_cast<const CompositeArrayField *>(field);
            referent = cafield->reflection;
        }

        if (referent == NULL) {
            continue;
        }

        if (std::find(references.begin(), references.end(), referent) != references.end())
            continue;
    result +=
    "#include \"core/xml/generated/" + toCamelCase(referent->name.name) + ".h\"\n";
        references.push_back(referent);
    }

    result +=
    "\n"
    "// using namespace corecvs;\n"
    "\n";

    /* Probably we don't need actual includes here.
     * Just use class definition */
    result +=
    "/*\n"
    " *  Additional includes for Pointer Types.\n"
    " */\n"
    "\n"
    "// namespace corecvs {\n";

    /* We don't bother vector is fast enough */
    vector<QString> pointedTypes;
    for (int i = 0; i < fieldNumber; i++ )
    {
        enterFieldContext(i);
        if (field->type != BaseField::TYPE_POINTER)
            continue;
        const PointerField *cfield = static_cast<const PointerField *>(field);
        QString target = QString(cfield->targetClass);
        if (std::find(pointedTypes.begin(), pointedTypes.end(), target) != pointedTypes.end())
            continue;
        QStringList namespaces = target.split("::");
        for (int n = 0; n < namespaces.size() - 1; n++)
        {
    result += "namespace " + namespaces[n] + " {\n";
        }

    result +=
    "class " + namespaces.last() + ";\n";

        for (int n = 0; n < namespaces.size() - 1; n++)
        {
    result += "}\n";
        }

        pointedTypes.push_back(target);
    }

    result +=
    "// }\n"
    "/*\n"
    " *  Additional includes for enum section.\n"
    " */\n";

    for (int i = 0; i < fieldNumber; i++ )
    {
        enterFieldContext(i);

        if (type != BaseField::TYPE_ENUM)
            continue;

        const EnumField *efield = static_cast<const EnumField *>(field);
        const EnumReflection *eref = efield->enumReflection;

        QString fileName = toCamelCase(eref->name.name) + ".h";

    result+=
    "#include \"" + fileName + "\"\n";

    }

    result+=
    "\n"
    "/**\n"
    " * \\brief "+classDescr+" \n"
    " * "+classComment+" \n"
    " **/\n"
    "class "+className+" : public corecvs::BaseReflection<"+className+">\n"
    "{\n"
    "public:\n"

    "    enum FieldId {\n";

    for (int i = 0; i < fieldNumber; i++ )
    {
        enterFieldContext(i);
    result+=
    "        "+fieldEnumId+",\n";

    }
    result+=
    "        "+classCapitalName+"_FIELD_ID_NUM\n"
    "    };\n"
    "\n"
    "    /** Section with variables */\n";

    for (int i = 0; i < fieldNumber; i++ )
    {
        enterFieldContext(i);

        if (type == BaseField::TYPE_ENUM)
            cppType = "int";

    result+=
    "\n"
    "    /** \n"
    "     * \\brief "+descr+" \n"
    "     * "+comment+" \n"
    "     */\n"
    "    "+cppType+" "+cppName+";\n";

        if (type == BaseField::TYPE_COMPOSITE)
        {
            PDOGenerator generator(static_cast<const CompositeField*>(field)->reflection);
            generator.generatePDOH();
            generator.generatePDOCpp();
            generator.generateControlWidgetCpp();
        }
        if (type == BaseField::TYPE_COMPOSITE_ARRAY)
        {
            PDOGenerator generator(static_cast<const CompositeArrayField*>(field)->reflection);
            generator.generatePDOH();
            generator.generatePDOCpp();
            generator.generateControlWidgetCpp();
        }

    }
    result+=
    "\n"
    "    /** Static fields init function, this is used for \"dynamic\" field initialization */ \n"
    "    static int staticInit();\n\n"
    "    static int relinkCompositeFields();\n\n"
    "    /** Section with getters */\n"
    "    const void *getPtrById(int fieldId) const\n"
    "    {\n"
    "        return (const unsigned char *)(this) + fields()[fieldId]->offset;\n"
    "    }\n";

    for (int i = 0; i < fieldNumber; i++ )
    {
        enterFieldContext(i);

        if (type == BaseField::TYPE_ENUM)
        {
            cppName = QString("static_cast<") + cppType + QString(">(") + cppName + ")";
        }

    result+=
    "    "+cppType+" "+getterName+"() const\n"
    "    {\n"
    "        return "+cppName+";\n"
    "    }\n"
    "\n";

    }
    result+=
    "    /* Section with setters */\n";
    for (int i = 0; i < fieldNumber; i++ )
    {
        enterFieldContext(i);

        QString ref = (type == BaseField::TYPE_COMPOSITE) ? "const &" : "";

    result+=
    "    void "+setterName+"("+cppType+" "+ref+getterName+")\n"
    "    {\n"
    "        "+cppName+" = "+getterName+";\n"
    "    }\n"
    "\n";

    }

    result+=
    "    /* Section with embedded classes */\n";
    for (int i = 0; i < embeddedNumber; i++ )
    {
        const EmbedSubclass *embed = clazz->embeds.at(i);
        const Reflection    *embedReflection = embed->subclass;
        name       = embed->name.name;
        comment    = embed->name.comment;
        descr      = embed->name.decription;
        cppType    = toCamelCase(embedReflection->name.name, true);
        getterName = toCamelCase(name, false);
        setterName = QString("set") + toCamelCase(name, true);
    result+=
    "    /** \n"
    "     * \\brief "+descr+" \n"
    "     * "+comment+" \n"
    "     */\n"
    "    "+cppType+" "+getterName+"() const\n"
    "    {\n"
    "        return "+cppType+"(\n";
        for (unsigned k = 0; k < embedReflection->fields.size(); k++)
        {
            const BaseField *field = embedReflection->fields.at(k);
            QString originalName = field->name.name;
            QString embeddedName = embed->getEmbeddedName(originalName.toLatin1().constData());

            QString fieldGetterName = toCamelCase(embeddedName, false);

            QString separator = (k == 0) ? " " : ",";
    result+=
    "           "+separator+" "+fieldGetterName+"()\n";
        }
    result+=
    "        );\n"
    "    }\n"
    "\n"
    "    void "+setterName+"("+cppType+" const &"+getterName+")\n"
    "    {\n";
        for (unsigned k = 0; k < embedReflection->fields.size(); k++)
        {
            const BaseField *field = embedReflection->fields.at(k);
            QString originalName = field->name.name;
            QString embeddedName = embed->getEmbeddedName(originalName.toLatin1().constData());
            QString fieldGetterName = toCamelCase(originalName, false);
            QString fieldCppName = QString("m") + toCamelCase(embeddedName, true);
    result+=
    "        "+j(fieldCppName,14)+" = "+getterName+"."+fieldGetterName+"();\n";
        }
    result+=
    "    }\n"
    "\n";

    }


    result+=
    "    /* visitor pattern - http://en.wikipedia.org/wiki/Visitor_pattern */\n"
    "template<class VisitorType>\n"
    "    void accept(VisitorType &visitor)\n"
    "    {\n";

    for (int i = 0; i < fieldNumber; i++ )
    {
        enterFieldContext(i);

    result+=
    "        visitor.visit("+ j(simplifiedType+cppName+",",27) + j(" static_cast<const corecvs::"+fieldRefType+" *>", 35)+"(fields()["+fieldEnumId+"]));\n";

    }

    result+=
    "    }\n"
    "\n"
    "    "+className+"()\n"
    "    {\n"
    "        corecvs::DefaultSetter setter;\n"
    "        accept(setter);\n"
    "    }\n"
    "\n";

    /** Constructor with fields **/
    result+=
    "    "+className+"(\n";

    for (int i = 0; i < fieldNumber; i++ )
    {
        enterFieldContext(i);
        QString separator = (i == 0) ? " " : ",";

    result+=
    "        "+separator+" "+cppType+" "+toCamelCase(name, false)+"\n";
    }

    result+=
    "    )\n"
    "    {\n";

    for (int i = 0; i < fieldNumber; i++ )
    {
        enterFieldContext(i);

    result+=
    "        "+cppName+" = "+toCamelCase(name, false)+";\n";
    }

    result+=
    "    }\n\n";
    /** Comparator **/
    result+=
    "    bool operator ==(const "+className+" &other) const \n"
    "    {\n";
    for (int i = 0; i < fieldNumber; i++ )
    {
        enterFieldContext(i);
        if (type == BaseField::TYPE_POINTER) {
            continue;
        }
    result+=
    "        if ( !(this->"+cppName+" == other."+cppName+")) return false;\n";
    }
    result+=
    "        return true;\n"
    "    }\n";

    result+=
    "    friend std::ostream& operator << (std::ostream &out, "+className+" &toSave)\n"
    "    {\n"
    "        corecvs::PrinterVisitor printer(out);\n"
    "        toSave.accept<corecvs::PrinterVisitor>(printer);\n"
    "        return out;\n"
    "    }\n"
    "\n"
    "    void print ()\n"
    "    {\n"
    "        std::cout << *this;\n"
    "    }\n";

    /* OGL style getter and setter */

    result+=
    "};\n"
    "#endif  //"+guardDefine+"\n";

    out << result.toLatin1().constData();
}

void PDOGenerator::generatePDOCpp()
{
    QString result;
    int fieldNumber = clazz->fields.size();

    QString fileName    = toCamelCase(clazz->name.name) + ".cpp";
    QString fileNameH    = toCamelCase(clazz->name.name) + ".h";
    QString guardDefine = QString(clazz->name.name).toUpper() + "_H_";
    QString className   = toCamelCase(clazz->name.name, true);

    QString classComment = clazz->name.comment;
    QString classDescr   = clazz->name.decription;

    out.close();
    out.open(QString(getGenerateDir() + QDir::separator() + fileName).toLatin1(), ios::out);

    result +=
    "/**\n"
    " * \\file "+fileName+"\n"
    " * \\attention This file is automatically generated and should not be in general modified manually\n"
    " *\n"
    " * \\date MMM DD, 20YY\n"
    " * \\author autoGenerator\n"
    " */\n"
    "\n"
    "#include <vector>\n"
    "#include <stddef.h>\n"
    "#include \""+fileNameH+"\"\n"
    "\n"
    "/**\n"
    " *  Looks extremely unsafe because it depends on the order of static initialization.\n"
    " *  Should check standard if this is ok\n"
    " *\n"
    " *  Also it's not clear why removing \"= Reflection()\" breaks the code;\n"
    " **/\n"
    "\n"
    "namespace corecvs {\n"
    "template<>\n"
    "Reflection BaseReflection<"+className+">::reflection = Reflection();\n"
    "template<>\n"
    "int BaseReflection<"+className+">::dummy = "+className+"::staticInit();\n"
    "} // namespace corecvs \n"
    "\n"
    "SUPPRESS_OFFSET_WARNING_BEGIN\n"
    "\n"
    "\n"
    "using namespace corecvs;\n"
    "\n"
    "int "+className+"::staticInit()\n"
    "{\n"
    "\n"
    "    ReflectionNaming &nameing = naming();\n"
    "    nameing = ReflectionNaming(\n"
    "        \"" + clazz->name.name + "\",\n"
    "        \"" + clazz->name.decription + "\",\n"
    "        \"" /*+ clazz->name.comment +*/ "\"\n"  //  We need to decorate comment
    "    );\n"
    "\n"
    "     getReflection()->objectSize = sizeof("+className+");\n"
    "     \n"
    "\n";

    for (int i = 0; i < fieldNumber; i++ )
    {
        enterFieldContext(i);

        if ( type == (BaseField::TYPE_DOUBLE | BaseField::TYPE_VECTOR_BIT))
        {
             const DoubleVectorField *vfield = static_cast<const DoubleVectorField *>(field);

    result+=
    "    double "+ cppName+"_dv[] = {";
             for (size_t val = 0; val < vfield->defaultSize; val++)
             {
    result+= (val == 0 ? "" : "," ) + QString::number(vfield->getDefaultElement(val));
             }
    result+="};\n";


        }

    result+=
    "    "+fieldRefType+"* "+fieldPlaceholder+" = new "+fieldRefType+"\n"
    "        (\n"
    "          "+className+"::"+fieldEnumId+",\n"
    "          offsetof("+className+", "+cppName+"),\n";

    if (type != BaseField::TYPE_COMPOSITE && type != BaseField::TYPE_COMPOSITE_ARRAY) {

        if (type == (BaseField::TYPE_DOUBLE | BaseField::TYPE_VECTOR_BIT))
        {
            const DoubleVectorField *vfield = static_cast<const DoubleVectorField *>(field);
            QString defaultSize = QString::number(vfield->defaultSize);
    result+=
    "          vector<double>("+cppName+"_dv, "+cppName+"_dv + "+defaultSize+"),\n"
    "          "+defaultSize+",\n";

        } else {

    result+=
    "          "+defaultValue+",\n";

        }

    result+=
    "          \""+name+"\",\n"
    "          \""+descr+"\",\n"
    "          \""+comment+"\"";

        if (type == BaseField::TYPE_INT) {
            const IntField *ifield = static_cast<const IntField *>(field);
            if (ifield->hasAdditionalValues)
            {
    result+=
    ",\n"
    "          true,\n"
    "         " + QString::number(ifield->min) + ",\n"
    "         " + QString::number(ifield->max);

            }
        }
        if (type == BaseField::TYPE_DOUBLE) {
            const DoubleField *dfield = static_cast<const DoubleField *>(field);
            if (dfield->hasAdditionalValues)
            {
    result+=
    ",\n"
    "          true,\n"
    "         " + QString::number(dfield->min) + ",\n"
    "         " + QString::number(dfield->max);

            }
        }


    } else if (type == BaseField::TYPE_COMPOSITE) {
        const CompositeField *cfield = static_cast<const CompositeField *>(field);
        const Reflection *referent = cfield->reflection;
    result+=
    "          \""+name+"\",\n"
    "          \""+toCamelCase(referent->name.name, true) + "\",\n"
    "          \""+descr+"\",\n"
    "          \""+comment+"\"";
    } else if (type == BaseField::TYPE_COMPOSITE_ARRAY) {
        const CompositeArrayField *cafield = static_cast<const CompositeArrayField *>(field);
        const Reflection *referent = cafield->reflection;
    result+=
    "          \""+name+"\",\n"
    "          \""+toCamelCase(referent->name.name, true) + "\",\n"
    "            "+QString::number(cafield->size)+",\n"
    "          \""+descr+"\",\n"
    "          \""+comment+"\"";
    }

    if (type == BaseField::TYPE_ENUM) {
        const EnumField *efield = static_cast<const EnumField *>(field);
        const EnumReflection *enumOptions = efield->enumReflection;
        result+=
                     ",\n";
        result+=
    "          new EnumReflection("+QString::number(enumOptions->options.size())+"\n" ;
        for(size_t enumCount = 0; enumCount < enumOptions->options.size(); enumCount++)
        {
            const EnumOption *option = enumOptions->options[enumCount];
            if(option->presentationHint == NULL)
            {
            result+=
    "          , new EnumOption("+QString::number(option->id)+",\""+option->name.name+"\")\n";
            } else {
                result+=
    "          , new EnumOption("+QString::number(option->id)+",\""+option->name.name+"\",\""+option->presentationHint+"\")\n";
            }
        }
        result+=
    "          )\n";
    } else if (type == BaseField::TYPE_COMPOSITE) {
        result+=
                     ",\n"
    "           NULL\n";
    } else if (type == BaseField::TYPE_POINTER) {
        result+=
                ",\n"
    "          \""+QString(static_cast<const PointerField *>(field)->targetClass)+"\"\n";
    } else {
        result += "\n";
    }

    result+=
    "        );\n";
    if (field->widgetHint != BaseField::DEFAULT_HINT)
       result+=
    "    "+fieldPlaceholder+"->widgetHint=BaseField::"+BaseField::getString(field->widgetHint)+";\n";
    if (field->prefixHint != NULL && strlen(field->prefixHint) != 0)
       result+=
    "    "+fieldPlaceholder+"->prefixHint=\""+field->prefixHint+"\";\n";
    if (field->suffixHint != NULL && strlen(field->suffixHint) != 0)
       result+=
    "    "+fieldPlaceholder+"->suffixHint=\""+field->suffixHint+"\";\n";

    if (field->precision >= 0)
        result+=
     "    "+fieldPlaceholder+"->precision="+QString::number(field->precision)+";\n";


    if (type == BaseField::TYPE_COMPOSITE) {
        const CompositeField *cfield = static_cast<const CompositeField *>(field);
        const Reflection *referent = cfield->reflection;
        result+=
    "    {\n"
    "        ReflectionDirectory* directory = ReflectionDirectoryHolder::getReflectionDirectory();\n"
    "        std::string name(\""+QString(referent->name.name)+"\");\n"
    "        ReflectionDirectory::iterator it = directory->find(name);\n"
    "        if(it != directory->end()) {\n"
    "             "+fieldPlaceholder+"->reflection = it->second;\n"
    "        } else {\n"
    "             printf(\"Reflection "+className+" to the subclass "+QString(referent->name.name)+" can't be linked\\n\");\n"
    "        }\n"
    "    }\n";

    }

    result+=
    "    fields().push_back("+fieldPlaceholder+");\n"
    "    /*  */ \n";
    }    

    result+=
    "    ReflectionDirectory &directory = *ReflectionDirectoryHolder::getReflectionDirectory();\n"
    "    directory[std::string(\""+QString(clazz->name.name)+"\")]= &reflection;\n";


    result+=
    "   return 0;\n"
    "}\n"
    "int "+className+"::relinkCompositeFields()\n"
    "{\n";
    for (int i = 0; i < fieldNumber; i++ )
    {
        enterFieldContext(i);
        if (type == BaseField::TYPE_COMPOSITE) {
            const CompositeField *cfield = static_cast<const CompositeField *>(field);
            const Reflection *referent = cfield->reflection;
        result+=
    "    {\n"
    "        ReflectionDirectory* directory = ReflectionDirectoryHolder::getReflectionDirectory();\n"
    "        std::string name(\""+QString(referent->name.name)+"\");\n"
    "        ReflectionDirectory::iterator it = directory->find(name);\n"
    "        if(it != directory->end()) {\n"
    "             const CompositeField* field = static_cast<const CompositeField*>(getReflection()->fields["+QString::number(i)+"]);\n"
    "             const_cast<CompositeField*>(field)->reflection = it->second;\n"
    "        } else {\n"
    "             printf(\"Reflection "+className+" to the subclass "+QString(referent->name.name)+" can't be linked\\n\");\n"
    "        }\n"
    "    }\n";

        }
    }
    result+=
    "   return 0;\n"
    "}\n"
    "\n"
    "SUPPRESS_OFFSET_WARNING_END\n"
    "\n"
    "\n";


    out << result.toLatin1().constData();
}


void PDOGenerator::generateControlWidgetCpp()
{
    QString result;
    int fieldNumber = clazz->fields.size();

    QString fileName = toCamelCase(clazz->name.name) + "ControlWidget.cpp";
    QString fileNameH = toCamelCase(clazz->name.name) + "ControlWidget.h";
    QString fileNameUi = "ui_" + toCamelCase(clazz->name.name) + "ControlWidget.h";
    QString className = toCamelCase(clazz->name.name, true) + "ControlWidget";
    QString parametersName = toCamelCase(clazz->name.name, true);

    const ReflectionGen *genClass = static_cast<const ReflectionGen *>(clazz);
    QString baseWidget = QString (genClass->uiBaseClass);
    if (baseWidget.isEmpty() || baseWidget.isNull())
        baseWidget = "ParametersControlWidgetBase";

    out.close();
    out.open(QString(getGenerateDir() + QDir::separator() + fileName).toLatin1(), ios::out);

    result +=
    "/**\n"
    " * \\file "+fileName+"\n"
    " * \\attention This file is automatically generated and should not be in general modified manually\n"
    " *\n"
    " * \\date MMM DD, 20YY\n"
    " * \\author autoGenerator\n"
    " */\n"
    "\n"
    "#include \""+fileNameH+"\"\n"
    "#include \""+fileNameUi+"\"\n"
	"#include <memory>\n"
    "#include \"qSettingsGetter.h\"\n"
    "#include \"qSettingsSetter.h\"\n"
    "\n";

    for (int i = 0; i < fieldNumber; i++ )
    {
        enterFieldContext(i);

        if (type == BaseField::TYPE_COMPOSITE)
        {
    result+=
    "#include \""+boxFileName+".h\"\n";
        }
    }

    result+=
    "\n"
    ""+className+"::"+className+"(QWidget *parent, bool _autoInit, QString _rootPath)\n"
    "    : "+baseWidget+"(parent)\n"
    "    , mUi(new Ui::"+className+")\n"
    "    , autoInit(_autoInit)\n"
    "    , rootPath(_rootPath)\n"
    "{\n"
    "    mUi->setupUi(this);\n"
    "\n";

    for (int i = 0; i < fieldNumber; i++)
    {
        enterFieldContext(i);
        if (type == BaseField::TYPE_DOUBLE)
        {
            const DoubleFieldGen *dfield = static_cast<const DoubleFieldGen *>(field);
            if (dfield->widgetHint == BaseField::SLIDER && dfield->max > 0)
            {
                result+=
                "mUi->"+boxName+"->setMaxZoom("+QString::number(dfield->max)+");\n"
                "\n";
            }
        }

    }

   /* result+=
    "    if (autoInit)\n"
    "    {\n"
    "          loadFromQSettings(\"cvs.conf\", rootPath);\n"
    "    }\n"
    "\n";*/

    for (int i = 0; i < fieldNumber; i++ )
    {
        enterFieldContext(i);
        if (type == BaseField::TYPE_POINTER)
            continue;
    result +=
    "    QObject::connect(mUi->"+boxName+", SIGNAL("+boxSignal+"), this, SIGNAL(paramsChanged()));\n";
    }

    result+=
    "}\n"
    "\n"
    ""+className+"::~"+className+"()\n"
    "{\n"
   /* "    if (autoInit)\n"
    "    {\n"
    "          saveToQSettings(\"cvs.conf\", rootPath);\n"
    "    }\n"*/
    "\n"
    "    delete mUi;\n"
    "}\n"
/*    "void "+className+"::loadFromQSettings(const QString &fileName, const QString &_root)\n"
    "{\n"
    "    "+parametersName+" *params = createParameters();\n"
    "    SettingsGetter visitor(fileName, _root + rootPath);\n"
    "    params->accept<SettingsGetter>(visitor);\n"
    "    setParameters(*params);\n"
    "    delete params;\n"
    "}\n"
    "\n"
    "void "+className+"::saveToQSettings  (const QString &fileName, const QString &_root)\n"
    "{\n"
    "    "+parametersName+" *params = createParameters();\n"
    "    SettingsSetter visitor(fileName, _root + rootPath);\n"
    "    params->accept<SettingsSetter>(visitor);\n"
    "    delete params;\n"
    "}\n"*/
    "\n"
    "void "+className+"::loadParamWidget(WidgetLoader &loader)\n"
    "{\n"
    "    std::unique_ptr<"+parametersName+"> params(createParameters());\n"
    "    loader.loadParameters(*params, rootPath);\n"
    "    setParameters(*params);\n"
    "}\n"
    "\n"
    "void "+className+"::saveParamWidget(WidgetSaver  &saver)\n"
    "{\n"
    "    saver.saveParameters(*std::unique_ptr<"+parametersName+">(createParameters()), rootPath);\n"
    "}\n"
    "\n"
    // Why we cannot use composite fields, while createParameters ()
    // works like a charm?!
#if 0
    " /* Composite fields are NOT supported so far */\n"
    "void "+className+"::getParameters("+parametersName+"& params) const\n"
    "{\n"
    "\n";

    for (int i = 0; i < fieldNumber; i++ )
    {
        enterFieldContext(i);
        if (type == BaseField::TYPE_COMPOSITE) {
            result += "//";
        }

        result+=
    "    params."+j(setterName,20)+"("+prefix+"mUi->"+boxName+"->"+getWidgetGetterMethodForType(type)+suffix+");\n";
    }
#else
	"void "+className+"::getParameters("+parametersName+"& params) const\n"
	"{\n"
	"    params = *std::unique_ptr<"+parametersName+">(createParameters());\n"
	"}\n"
	"\n";
#endif
    result+=
    "\n"
    ""+parametersName+" *"+className+"::createParameters() const\n"
    "{\n"
    "\n"
    "    /**\n"
    "     * We should think of returning parameters by value or saving them in a preallocated place\n"
    "     **/\n"
    "\n";
    
    result+=
    "\n"
    "    return new "+parametersName+"(\n";

    for (int i = 0; i < fieldNumber; i++ )
    {
        enterFieldContext(i);

        if (type == BaseField::TYPE_COMPOSITE)
        {
            prefix = "*std::unique_ptr<" + cppType + ">(";
            suffix = ")";
        }

        QString separator = (i == 0) ? " " : ",";

        if (type == BaseField::TYPE_POINTER)
        {
	result+=
    "        "+separator+" NULL\n";
            continue;
        }


    result+=
    "        "+separator+" "+prefix+"mUi->"+boxName+"->"+getWidgetGetterMethodForType(type)+suffix+"\n";
    }

    result+=
    "    );\n"
    "}\n"
    "\n"
    "void "+className+"::setParameters(const "+parametersName+" &input)\n"
    "{\n"
    "    // Block signals to send them all at once\n"
    "    bool wasBlocked = blockSignals(true);\n";

    for (int i = 0; i < fieldNumber; i++ )
    {
        enterFieldContext(i);

        if (type == BaseField::TYPE_POINTER)
            continue;
        if       (type == BaseField::TYPE_STRING)
        {
    result+=
    "    "+prefix+"mUi->"+boxName+"->"+getWidgetSetterMethodForType(type)+"(QString::fromStdString(input."+getterName+"()));\n";
        } else if(type == BaseField::TYPE_WSTRING)
        {
    result+=
    "    "+prefix+"mUi->"+boxName+"->"+getWidgetSetterMethodForType(type)+"(QString::fromStdWString(input."+getterName+"()));\n";
        } else {

            if (type == BaseField::TYPE_ENUM)
            {
                prefix = "";
            }

    result+=
    "    "+prefix+"mUi->"+boxName+"->"+getWidgetSetterMethodForType(type)+"(input."+getterName+"());\n";
        }

    }

    result+=
    "    blockSignals(wasBlocked);\n"
    "    emit paramsChanged();\n"
    "}\n";



    result+=
    "\n"
    "void "+className+"::setParametersVirtual(void *input)\n"
    "{\n"
    "    // Modify widget parameters from outside\n"
    "    " + parametersName + " *inputCasted = static_cast<"+parametersName+" *>(input);\n"
    "    setParameters(*inputCasted);\n"
    "}\n";



    out << result.toLatin1().constData();

}

