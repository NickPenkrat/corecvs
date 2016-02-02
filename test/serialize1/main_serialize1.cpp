#include <stdio.h>
#ifndef WIN32
#include <unistd.h>
#endif
#include <QtXml/QDomDocument>
#include <vector>

#include "fixtureScene.h"

#include "vector3d.h"
#include "xmlSetter.h"
#include "xmlGetter.h"

#include "jsonGetter.h"
#include "jsonSetter.h"

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


/*========== Now with json ==============*/


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
    SYNC_PRINT(("Parsing %zu input:\n%s\n", CORE_COUNT_OF(input), input));


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
        CORE_ASSERT_TRUE_P(testObject1.testField[i] == testObject.testField[i], ("Error at pos %zu", i));
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

void testJSON_FixtureScene()
{
    cout << "----------------Running the test-------------" << std::endl;
    FixtureScene *scene = new FixtureScene();

    CameraFixture *fixture1 = scene->createCameraFixture();
    fixture1->name = "Fixture1";
    CameraFixture *fixture2 = scene->createCameraFixture();
    fixture2->name = "Fixture2";
    CameraFixture *fixture3 = scene->createCameraFixture();
    fixture3->name = "Fixture3";

    //SYNC_PRINT(("%p %p %p\n", fixture1, fixture2, fixture3));
    //SYNC_PRINT(("%p %p %p\n", scene->fixtures[0], scene->fixtures[1], scene->fixtures[2]));

    //SYNC_PRINT(("Length: %d\n", scene->fixtures.size()));

    fixture1->setLocation(Affine3DQ(Vector3dd( 0, 0,-10)));
    fixture2->setLocation(Affine3DQ(Vector3dd(50, 0,  0)));
    fixture3->setLocation(Affine3DQ(Vector3dd( 0,50, 30)));

    for (int j = 0; j < 5; j++)
    {

        CameraModel model;
        double angle = degToRad(360.0 / 5 * j);
        CameraLocationAngles ang(angle, 0.0, 0.0);
        Affine3DQ position(Quaternion::RotationZ(angle), Vector3dd::FromCylindrical(angle, 5.0, 0.0));

        model.extrinsics = CameraLocationData(position);
        model.intrinsics.principal.x() = 100;
        model.intrinsics.principal.y() = 100;

        model.intrinsics.focal.x() = 100;
        model.intrinsics.focal.y() = 100;

        model.intrinsics.size = Vector2dd(200, 200);

        //SYNC_PRINT(("Length: %d\n", scene->fixtures.size()));

        for (size_t i = 0; i < scene->fixtures.size(); i++)
        {
            CameraFixture *fixture = scene->fixtures[i];
            FixtureCamera *camera = scene->createCamera();
            char buffer[100];
            snprintf2buf(buffer, "camera %d(%d)", j, i);
            camera->nameId = buffer;
            camera->copyModelFrom(model);
            //scene->positionCameraInFixture(fixture, camera, camera->extrinsics.toAffine3D());
            //scene->positionCameraInFixture(fixture, camera, position);
            scene->positionCameraInFixture(fixture, camera, position);

            //SYNC_PRINT(("Adding camera %s to fixture %d %p\n", buffer, i, fixture));

            scene->addCameraToFixture(camera, fixture);

        }

    }

    for (int i = 0; i < 5; i++)
    {
        SceneFeaturePoint *point = scene->createFeaturePoint();
        point->setPosition(Vector3dd::FromCylindrical(degToRad(i * (360 / 5)), 130.0, 0.5));
        point->color = RGBColor::rainbow(i / (5.0 - 1));
    }

    scene->projectForward(SceneFeaturePoint::POINT_ALL);

    cout << "Original scene:" << endl;
    cout << "================================" << endl;
    scene->dumpInfo(cout);
    cout << "================================" << endl;

    {
        JSONSetter setter("scene.json");
        setter.visit(*scene, "scene");
    }
    delete_safe(scene);



    /** Now loading **/
    FixtureScene *loaded = new FixtureScene();
    {
        JSONGetter getter("scene.json");
        getter.visit(*loaded, "scene");
    }

    cout << "Loaded scene:" << endl;
    cout << "================================" << endl;
    loaded->dumpInfo(cout);
    cout << "================================" << endl;
}



int main (int /*argc*/, char ** /*argv*/)
{
    printf("Quick test\n");

//    testXML1();
//    testXML2();

//    testJSON1();
//    testJSON2();
//    testJSON_UInt64();
    testJSON_FixtureScene();


	return 0;
}
