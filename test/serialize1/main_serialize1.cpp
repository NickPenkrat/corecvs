#include <stdio.h>
#ifndef WIN32
#include <unistd.h>
#endif
#include <QtXml/QDomDocument>
#include <vector>

#include "abstractPainter.h"
#include "bmpLoader.h"

#include "fixtureScene.h"

#include "vector3d.h"
#include "xmlSetter.h"
#include "xmlGetter.h"

#include "jsonGetter.h"
#include "jsonSetter.h"

#ifdef WITH_RAPIDJSON
    #include "rapidJSONReader.h"
#endif


#ifdef WITH_JSONMODERN
    #include "jsonModernReader.h"
#endif

/*******************************
 *    XML
 ********************************/

void testXML1()
{
    QDomDocument document("document");
    //QDomElement mDocumentElement = document.createElement("xml");
    //QDomElement mDocumentElement = document; //document.documentElement();

    //QDomElement mDocumentElement1 = mDocumentElement;
    QDomElement newElement = document.createElement("tag");
    newElement.setAttribute("value", "test");
    QDomNode node = document.appendChild(newElement);

    qDebug() << document.toString();
}

void testXML2()
{

    printf("Serializing some data\n");

    Vector3dd someData(1.0,2.0,5.0);
    {
        XmlSetter setter("output.xml");
        setter.visit(someData, "This_is_some_data");
    }

    printf("====== Loading back some data ======\n");

    Vector3dd result(1.5,2.5,5.5);
    {
        XmlGetter getter("output.xml");
        getter.visit(result, "This_is_some_data");
    }

    cout << "Result:" << result;
}


/*******************************
 *    Now with QT json
 ********************************/

void testJSON1()
{
    const char input[] =
    "{\n"
    "  \"vector\":{\n"
    "     \"x\": 1,\n"
    "     \"y\": 2,\n"
    "     \"z\": 3\n"
    "  }\n"
    "}\n";



   /* printf("Serializing some data\n");

    Vector3dd someData(1.0,2.0,5.0);
    {
        XmlSetter setter("output.xml");
        setter.visit(someData, "This_is_some_data");
    }

    printf("====== Loading back some data ======\n");
*/
    SYNC_PRINT(("Parsing %" PRISIZE_T " input:\n%s\n", CORE_COUNT_OF(input), input));


    QByteArray array(input, CORE_COUNT_OF(input));
    SYNC_PRINT(("Array:\n%s\n", array.constData()));


    QJsonDocument document = QJsonDocument::fromJson(array);
    if (document.isNull())
    {
         SYNC_PRINT(("Fail parsing the data"));
    }

    QJsonObject  object = document.object();

    Vector3dd result(1.5,2.5,5.5);
    {
        JSONGetter getter(object);
        getter.visit(result, "vector");
    }

    /*======== Now saving ===========*/
    {
        JSONSetter setter("out.txt");
        setter.visit(result, "vector");
    }

    cout << "Result:" << result;
}


class TestComplexStructure : public BaseReflection<TestComplexStructure>{
public:
    std::vector<double> testField;

    static int staticInit();

    template<class VisitorType>
        void accept(VisitorType &visitor)
        {
            visitor.visit(testField,  static_cast<const DoubleVectorField *>(fields()[0]));
        }

};

template<>
Reflection BaseReflection<TestComplexStructure>::reflection = Reflection();
template<>
int BaseReflection<TestComplexStructure>::dummy = TestComplexStructure::staticInit();

int TestComplexStructure::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming("TestComplexStructure");

    fields().push_back(
        new DoubleVectorField
        (
          0,
          offsetof(TestComplexStructure, testField),
          0,
          0,
          "testField",
          "testField",
          "testField"
        )
    );
   return 0;
}

void testJSON2()
{
    TestComplexStructure testObject;
    for (int i = 0; i < 7; i++) {
        testObject.testField.push_back(i / 7.8);
    }

    /*======== Now saving ===========*/
    {
        JSONSetter setter("out.txt");
        setter.visit(testObject, "vector");
    }

    /*======== And loading back =======*/
    TestComplexStructure testObject1;
    {
        JSONGetter getter("out.txt");
        getter.visit(testObject1, "vector");
    }


    CORE_ASSERT_TRUE(testObject.testField.size() == testObject1.testField.size(), "Wrong loaded array size");
    for (size_t i = 0; i < testObject.testField.size(); i++)
    {
        CORE_ASSERT_TRUE_P(testObject1.testField[i] == testObject.testField[i], ("Error at pos %" PRISIZE_T "", i));
        cout << testObject1.testField[i] << " == " << testObject.testField[i] << std::endl;
    }
    cout << std::endl;

}

/*======================================*/
class Uint64Structure : public BaseReflection<TestComplexStructure>{
public:
    uint64_t testField1;
    uint64_t testField2;

    static int staticInit();

    template<class VisitorType>
        void accept(VisitorType &visitor)
        {
            visitor.visit(testField1,  static_cast<const Int64Field *>(fields()[0]));
            visitor.visit(testField2,  (uint64_t)0xDEADBEEF, "test1" );
        }

};

template<>
Reflection BaseReflection<Uint64Structure>::reflection = Reflection();
template<>
int BaseReflection<Uint64Structure>::dummy = TestComplexStructure::staticInit();

int Uint64Structure::staticInit()
{

    ReflectionNaming &nameing = naming();
    nameing = ReflectionNaming("TestComplexStructure");

    fields().push_back(
        new Int64Field
        (
          0,
          offsetof(TestComplexStructure, testField),
          0,
          0,
          "testField",
          "testField",
          "testField"
        )
    );
   return 0;
}

void testJSON_UInt64()
{
    Uint64Structure testObject;
    testObject.testField1 = 0xF000F000E000E000lu;
    testObject.testField2 = 700;

    cout << "In   1" << std::hex << testObject.testField1 << std::endl;
    cout << "In   2" << std::hex << testObject.testField2 << std::endl;


    /*======== Now saving ===========*/
    {
        JSONSetter setter("out.txt");
        setter.visit(testObject, "test64");
    }

    /*======== And loading back =======*/
    Uint64Structure testObject1;
    {
        JSONGetter getter("out.txt");
        getter.visit(testObject1, "test64");
    }


    cout << "Orig 1" << std::hex << testObject.testField1 << std::endl;
    cout << "Orig 2" << std::hex << testObject.testField2 << std::endl;

    cout << "Load 1" << std::hex << testObject1.testField1 << std::endl;
    cout << "Load 2" << std::hex << testObject1.testField2 << std::endl;


}


void testJSON_saveDistortion()
{
    LensDistortionModelParameters params;

    params.setKoeff({1.0, std::numeric_limits<double>::min()});

    cout << "testJSON_saveDistortion() test called" << endl;
    cout << "Initial state" << endl;
    cout << params << endl;

    {
        JSONSetter saver("out1.json");
        saver.visit(params, "stage1");
    }
    {
        JSONGetter loader("out1.json");
        loader.visit(params, "stage1");
    }
    {
        JSONSetter saver("out2.json");
        saver.visit(params, "stage2");
    }

    LensDistortionModelParameters loaded;
    {
        JSONGetter loader("out2.json");
        loader.visit(loaded, "stage2");
    }
    cout << "After loading" << endl;
    cout << loaded << endl;

    cout << "Exiting" << endl;


}

#ifdef WITH_RAPIDJSON
/*******************************
 *    Now with Rapid json
 ********************************/


void testRapidJSON1()
{
    cout << std::endl;
    cout << "testRapidJSON1()" << std::endl;
    cout << std::endl;

    const char input[] =
    "{\n"
    "  \"vector\":{\n"
    "     \"x\": 1,\n"
    "     \"y\": 2,\n"
    "     \"z\": 3\n"
    "  }\n"
    "}\n";

    SYNC_PRINT(("Parsing %" PRISIZE_T "bytes input:\n%s\n", CORE_COUNT_OF(input), input));

    rapidjson::Document document;
    document.Parse(input);
    rapidjson::Value object = document.GetObject();

    cout << "Before parsing" << endl;
    Vector3dd result(1.5,2.5,5.5);
    cout << result << endl;
    {
        RapidJSONReader getter(object);
        getter.visit(result, "vector");
    }
    cout << "After parsing" << endl;
    cout << result << endl;

}

void testRapidJSON_saveDistortion()
{
    cout << std::endl;
    cout << "testRapidJSON_saveDistortion()" << std::endl;
    cout << std::endl;

    LensDistortionModelParameters params;
    LensDistortionModelParameters loaded;

    params.setPrincipalX(400);
    params.setPrincipalY(400);
    params.setTangentialX(0.3);
    params.setTangentialY(0.4);

    params.setShiftX(200);
    params.setShiftY(200);

    params.setAspect(10.0);
    params.setScale(1.1);
    params.setKoeff({1.0, 2.0, std::numeric_limits<double>::min()});

    cout << "testJSON_saveDistortion() test called" << endl;
    cout << "Initial state" << endl;
    cout << params << endl;

    {
        JSONSetter saver("out1.json");
        saver.visit(params, "stage1");
    }
    {
        RapidJSONReader loader("out1.json");
        loader.visit(loaded, "stage1");
    }

    cout << "After parsing" << endl;
    cout << loaded << endl;

}
#endif

#ifdef WITH_JSONMODERN
/*******************************
 *    Now with json modern
 ********************************/


void testJSONModern1()
{
    cout << std::endl;
    cout << "testRapidJSON1()" << std::endl;
    cout << std::endl;

    const char input[] =
    "{\n"
    "  \"vector\":{\n"
    "     \"x\": 1,\n"
    "     \"y\": 2,\n"
    "     \"z\": 3\n"
    "  }\n"
    "}\n";

    SYNC_PRINT(("Parsing %" PRISIZE_T "bytes input:\n%s\n", CORE_COUNT_OF(input), input));

    nlohmann::json document = nlohmann::json::parse(input);
    cout << "Before parsing" << endl;
    Vector3dd result(1.5,2.5,5.5);
    cout << result << endl;
    {
        JSONModernReader getter(document);
        getter.visit(result, "vector");
    }
    cout << "After parsing" << endl;
    cout << result << endl;

}

void testJSONModern_saveDistortion()
{
    cout << std::endl;
    cout << "testRapidJSON_saveDistortion()" << std::endl;
    cout << std::endl;

    LensDistortionModelParameters params;
    LensDistortionModelParameters loaded;

    params.setPrincipalX(400);
    params.setPrincipalY(400);
    params.setTangentialX(0.3);
    params.setTangentialY(0.4);

    params.setShiftX(200);
    params.setShiftY(200);

    params.setAspect(10.0);
    params.setScale(1.1);
    params.setKoeff({1.0, 2.0, std::numeric_limits<double>::min()});

    cout << "testJSON_saveDistortion() test called" << endl;
    cout << "Initial state" << endl;
    cout << params << endl;

    {
        JSONSetter saver("out1.json");
        saver.visit(params, "stage1");
    }
    {
        JSONModernReader loader("out1.json");
        loader.visit(loaded, "stage1");
    }

    cout << "After parsing" << endl;
    cout << loaded << endl;

}
#endif


int main (int /*argc*/, char ** /*argv*/)
{
    printf("Quick test\n");

//    testXML1();
//    testXML2();

//    testJSON1();
//    testJSON2();
//    testJSON_UInt64();
//    testJSON_saveDistortion();

    /*testJSON_FixtureScene();*/
    // testJSON_StereoScene();

#ifdef WITH_RAPIDJSON
     testRapidJSON1();
     testRapidJSON_saveDistortion();
#endif

#ifdef WITH_JSONMODERN
     testJSONModern1();
     testJSONModern_saveDistortion();
#endif

	return 0;
}
