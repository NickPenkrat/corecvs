#include "gtest/gtest.h"

#include "indoorHelper.h"
#include "indoorSolver.h"

TEST(INDOORS_SOLVER, checkIfSolverWorking)
{
    Affine3DQ simulatedLocation = Affine3DQ(
        Quaternion::Rotation(Vector3dd(0.0, 1.0, 0.0), degToRad(10)),
        Vector3dd(3.0, 1.0, -1.0)
    );

    FixtureScene *scene = IndoorHelper::FormTestScene(simulatedLocation);

    scene->projectForward(SceneFeaturePoint::POINT_ALL);

    for (int costType = 0; costType < IndoorSolver::COST_LAST; costType++)
    {
        IndoorSolver solver((IndoorSolver::CostFunction)costType);
        solver.trace = true;
        solver.scene = scene;
        solver.solve();

        cout << "Cost function score at the right position is " <<  solver.resultScore << endl;
        ASSERT_GE(solver.resultScore, 0);
    }
}

TEST(INDOORS_SOLVER, solverAtZeroPos)
{
    Affine3DQ simulatedLocation = Affine3DQ(
        Quaternion::Rotation(Vector3dd(0.0, 1.0, 0.0), degToRad(10)),
        Vector3dd(3.0, 1.0, -1.0)
    );

    FixtureScene *scene = IndoorHelper::FormTestScene(simulatedLocation);

    scene->projectForward(SceneFeaturePoint::POINT_ALL);

    Vector3dd  initialPos = Vector3dd(3.1, 1.1, -1.3);
    Quaternion initialRot = Quaternion::Rotation(Vector3dd(0.2, 1.1, 0.1), degToRad(8));


    for (int costType = IndoorSolver::COST_RAY_DIFFERENCE; costType < IndoorSolver::COST_LAST; costType++)
    {
        scene->fixtures[0]->setLocation(CameraLocationData(initialPos, initialRot));

        IndoorSolver solver((IndoorSolver::CostFunction)costType);
        solver.scene = scene;
        solver.trace = true;


        solver.solve();

        //solver.currentCost();

        cout << "Cost " << costType << " function score at the answer position is " <<  solver.resultScore << endl;

        ASSERT_GE(solver.resultScore, 0);
    }
}

TEST(INDOORS_SOLVER, solverStability)
{
    Affine3DQ simulatedLocation = Affine3DQ(
            Quaternion::Rotation(Vector3dd(0.0, 1.0, 0.0), degToRad(10)),
            Vector3dd(3.0, 1.0, -1.0)
    );

    FixtureScene *scene = IndoorHelper::FormTestScene(simulatedLocation);

    /* Fix observations */
    cout << "Making the projection" << endl;
    scene->projectForward(SceneFeaturePoint::POINT_ALL, true);
    //cout << endl;


    /* Reset photostation position to somewhere near */

    for (double z = -4.0; z <= 4.0; z+=2 )
    {
        for (double x = -10.0; x <= 10.0; x+=4 )
        {
            for (double y = -10.0; y <= 10.0; y+=4 )
            {
                for (double a = 0; a < 360.0; a+= 45.0 )
                {
                    for (double b = -30.0; b < 31.0; b+= 30.0 )
                    {

                        Vector3dd  initialPos = Vector3dd(x, z, y);
                        Quaternion initialRot = Quaternion::RotationY(degToRad(a + 7.0 )) ^ Quaternion::RotationX(degToRad(b));
                        scene->fixtures[0]->setLocation(Affine3DQ(initialRot, initialPos));

                        cout << "Looking for:" << endl;
                        simulatedLocation.prettyPrint1();
                        cout << endl;

                        cout << "Guess:" << endl;
                        Affine3DQ(initialRot, initialPos).prettyPrint1();
                        cout << endl;

                        IndoorSolver solver(IndoorSolver::COST_RAY_DIFFERENCE);
                        solver.scene = scene;
                        solver.trace = true;
                        solver.solve();

                        Affine3DQ answer = scene->fixtures[0]->location;

                        Vector3dd  posDiff =  answer.shift - simulatedLocation.shift;
                        Quaternion rotDiff = (answer.rotor ^ simulatedLocation.rotor.conjugated());
                        rotDiff = rotDiff.positivised().normalised();

                        double rTolerance = 1.0 / 60.0 / 6.0;   // 10seconds
                        double pTolerance = 0.01;               // 1cm

                        bool ok = false;
                        if (posDiff.notTooFar(Vector3dd(0.0), pTolerance) &&
                            fabs(rotDiff.getAngle()) < degToRad(rTolerance))
                        {
                            ok = true;
                        } else {
                            cout << "Failure" << endl;
                            cout << "Shift:\n" << answer.shift << endl << simulatedLocation.shift << endl << posDiff << endl << "---\n";

                            cout << "Rotor:\n" << answer.rotor << endl << simulatedLocation.rotor << endl << rotDiff << endl << "---\n";
                            cout << "Diff = " << rotDiff.getAngle() << "rad\n";
                            cout << "Expectation = " << degToRad(rTolerance) << "rad\n";
                        }

                        SYNC_PRINT(("% 3.2lf % 3.2lf % 3.2lf (% 3.2lf % 3.2lf) --> %s [%lg] [%.2lf %.2lf %.2lf] [%lf]\n",
                                     z,x,y,a,b,
                                    (ok ? "O" : "P"),
                                    solver.resultScore,
                                    posDiff.x(), posDiff.y(), posDiff.z(),
                                    rotDiff.getAngle()
                                    ));
                        ASSERT_TRUE(ok);
                    }
                }
            }
        }
    }
}

struct StatsReport {
    int slice;

    Affine3DQ location;
    Affine3DQ answer;

    Vector3dd posErr;
    double angle;
    int hasVisible;

    double score;

    RGBColor colorCode;
};

TEST(INDOORS_SOLVER, solverPrecision)
{
    int totalok = 0;
    int totalfail = 0;
    int totalalmost = 0;

    FixtureScene *scene = NULL;


    vector<StatsReport> report;

    int sliceId = 0;
  //  vector<Mesh3D> slices;

    for (double z = -4.0; z <= 4.0; z+=2.0, sliceId++ )
    {
    /*    slices.push_back(Mesh3D);
        Mesh3D &sliceMesh = slices.back();
        sliceMesh.switchColor();*/

        for (double x = -75.0; x <= 75.0; x+=5.0 )
        {
            for (double y = -75.0; y <= 75.0; y+=5.0 )
            {
                for (double a = 0; a < 360.0; a+= 45.0 )
                {
                    for (double b = -20.0; b < 21.0; b+= 20.0 )
                    {

                        StatsReport r;
                        Vector3dd toGuess(x, z, y);

                        Affine3DQ simulatedLocation = Affine3DQ(
                            Quaternion::RotationY(degToRad(a + 7.0 )) ^ Quaternion::RotationX(degToRad(b)),
                            toGuess
                        );

                        delete_safe(scene);
                        scene = IndoorHelper::FormTestScene1(simulatedLocation);
                        scene->projectForward(SceneFeaturePoint::POINT_ALL, true);

                        Vector3dd  initialPos = Vector3dd(0.0, 0.0, 0.0);
                        Quaternion initialRot = Quaternion::RotationIdentity();
                        scene->fixtures[0]->setLocation(Affine3DQ(initialRot, initialPos));

                        IndoorSolver solver(IndoorSolver::COST_RAY_DIFFERENCE);
                        solver.scene = scene;
                        solver.solve();

                        Affine3DQ answer = scene->fixtures[0]->location;

                        Vector3dd  posDiff =  answer.shift - simulatedLocation.shift;
                        Quaternion rotDiff = (answer.rotor ^ simulatedLocation.rotor.conjugated());
                        rotDiff = rotDiff.positivised().normalised();

                        r.slice = sliceId;
                        r.location = simulatedLocation;
                        r.answer   = answer;
                        r.posErr = posDiff;
                        r.angle = rotDiff.getAngle();

                        int obs = (int)scene->totalObservations();
                        r.hasVisible = obs;
                        r.score = solver.resultScore;

                        report.push_back(r);

                        if (y == 0.0)
                        SYNC_PRINT(("% 3.2lf % 3.2lf % 3.2lf (% 3.2lf % 3.2lf) --> [%lg] [%.2lf %.2lf %.2lf] [%lf deg] [%d points visible]\n",
                                    z,x,y,a,b,
                                    solver.resultScore,
                                    posDiff.x(),
                                    posDiff.y(),
                                    posDiff.z(),
                                    radToDeg(rotDiff.getAngle()),
                                    obs
                                    ));
                    }
                }
            }
        }
    }

    enum {
        TOTAL_TESTS,
        PASSED_TESTS,
        ALMOST_TESTS,
        FAILED_TESTS,
        STAT_NUM
    };

    int visibilityFull  [6][STAT_NUM];
    int visibilityCircle[6][STAT_NUM];

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < STAT_NUM; j++)
        {
            visibilityCircle[i][j] = 0;
            visibilityFull  [i][j] = 0;
        }
    }


    double rTolerance = 600.0 / 60.0 / 60.0;  // 5second
    double pTolerance = 0.02;               // 2cm

    double rAllowence = 2.0;  // 2 degree
    double pAllowence = 0.2;  // 20cm

    int total       = 0;
    int totalCircle = 0;

    //std::ofstream f("stats.txt");
    std::ostream &f = std::cout;

    for (StatsReport &r: report) {
        bool ok = false;
        bool almost = false;

        if ((r.posErr.l2Metric() < pTolerance) && fabs(r.angle) < degToRad(rTolerance))
        {
            ok = true;
            totalok++;
        } else {
            cout << r.answer.rotor << " " << r.location.rotor << " " << r.angle << endl;

            if ((r.posErr.l2Metric() < pAllowence) && fabs(r.angle) < degToRad(rAllowence))
            {
                almost = true;
                totalalmost++;
            } else {
                totalfail++;
            }
        }

        const char *sym = "O";

        if (ok) {
            r.colorCode = RGBColor::Green(); sym = "O";
        } else if (almost) {
            r.colorCode = RGBColor::Blue();  sym = "A";
        } else {
            r.colorCode = RGBColor::Red();   sym = "F";
        }

        visibilityFull[r.hasVisible][TOTAL_TESTS]++;
        total++;
        if (ok    ) visibilityFull[r.hasVisible][PASSED_TESTS]++;
        if (almost) visibilityFull[r.hasVisible][ALMOST_TESTS]++;
        if (!ok && !almost) visibilityFull[r.hasVisible][FAILED_TESTS]++;

        if (r.location.shift.l2Metric() <= 25.0) {
            visibilityCircle[r.hasVisible][TOTAL_TESTS]++;
            totalCircle++;
            if (ok    ) visibilityCircle[r.hasVisible][PASSED_TESTS]++;
            if (almost) visibilityCircle[r.hasVisible][ALMOST_TESTS]++;
            if (!ok && !almost) visibilityCircle[r.hasVisible][FAILED_TESTS]++;

        }

        f << r.location.shift.l2Metric() << " " << r.posErr.l2Metric() << " " << r.angle << " " << r.hasVisible << " " <<
             r.location.shift.x() << " " << r.location.shift.y() << " " << r.location.shift.z() << " " <<
             r.posErr.x() << " " << r.posErr.y() << " " << r.posErr.z() << endl;
   }

    //f.close();


    cout << "Total:"  << total << endl;
    for (size_t i = 0; i < CORE_COUNT_OF(visibilityFull); i++)
    {
        cout << "Visible:" << i << " -> " << visibilityFull[i][TOTAL_TESTS] << endl;
        cout << "       Passed:" << visibilityFull[i][PASSED_TESTS] << endl;
        cout << "       Almost:" << visibilityFull[i][ALMOST_TESTS] << endl;
        cout << "       Failed:" << visibilityFull[i][FAILED_TESTS] << endl;

    }
    cout << "Visible 4-5:" << visibilityFull[4][TOTAL_TESTS] + visibilityFull[5][TOTAL_TESTS] << endl;

    cout << "Ok    :"  << totalok << endl;
    cout << "Almost:"  << totalalmost << endl;
    cout << "Fail  :"  << totalfail << endl;

    ASSERT_GE(totalok,   55951);
    ASSERT_LE(totalfail, 38629);
    ASSERT_LE(totalalmost, 20740);

    cout << endl;
    cout << endl;

    for (size_t i = 0; i < CORE_COUNT_OF(visibilityFull); i++)
    {
        cout << i << " " << visibilityFull[i][TOTAL_TESTS] << " ";
        cout << visibilityFull[i][PASSED_TESTS] << "(" << (100.0 * visibilityFull[i][PASSED_TESTS] / visibilityFull[i][TOTAL_TESTS]) << "%) ";
        cout << visibilityFull[i][ALMOST_TESTS] << "(" << (100.0 * visibilityFull[i][ALMOST_TESTS] / visibilityFull[i][TOTAL_TESTS]) << "%) ";
        cout << visibilityFull[i][FAILED_TESTS] << "(" << (100.0 * visibilityFull[i][FAILED_TESTS] / visibilityFull[i][TOTAL_TESTS]) << "%) " << endl;

    }

    cout << endl;
    cout << endl;

    cout << "Total Circle:"  << totalCircle << endl;
    for (size_t i = 0; i < CORE_COUNT_OF(visibilityCircle); i++)
    {
        cout << "Visible:" << i << " -> " << visibilityCircle[i][TOTAL_TESTS] << endl;
        cout << "       Passed:" << visibilityCircle[i][PASSED_TESTS] << endl;
        cout << "       Almost:" << visibilityCircle[i][ALMOST_TESTS] << endl;
        cout << "       Failed:" << visibilityCircle[i][FAILED_TESTS] << endl;

    }
    cout << "Visible 4-5:" << visibilityCircle[4][TOTAL_TESTS] + visibilityCircle[5][TOTAL_TESTS] << endl;

    cout << endl;
    cout << endl;

    for (size_t i = 0; i < CORE_COUNT_OF(visibilityCircle); i++)
    {
        cout << i << " " << visibilityCircle[i][TOTAL_TESTS] << " ";
        cout << visibilityCircle[i][PASSED_TESTS] << "(" << (100.0 * visibilityCircle[i][PASSED_TESTS] / visibilityCircle[i][TOTAL_TESTS]) << "%) ";
        cout << visibilityCircle[i][ALMOST_TESTS] << "(" << (100.0 * visibilityCircle[i][ALMOST_TESTS] / visibilityCircle[i][TOTAL_TESTS]) << "%) ";
        cout << visibilityCircle[i][FAILED_TESTS] << "(" << (100.0 * visibilityCircle[i][FAILED_TESTS] / visibilityCircle[i][TOTAL_TESTS]) << "%) " << endl;
        if(i == 4 || i == 5)
            ASSERT_TRUE(visibilityCircle[i][PASSED_TESTS] / visibilityCircle[i][TOTAL_TESTS]);
    }
}
