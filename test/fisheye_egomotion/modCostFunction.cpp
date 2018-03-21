#include "modCostFunction.h"

void MODCostFunction::operator()(const double in[], double out[])
{
    if (data == NULL) {
        return;
    }

    /* Prepare */
    OmnidirectionalProjection model = data->projection;
    model.mN.resize(MODCostFunctionData::OMNIDIR_POWER);
    memcpy(model.mN.data(), in, sizeof(double) * MODCostFunctionData::OMNIDIR_POWER);

    for (size_t i = 0; i < data->obseravtions.size(); i++)
    {
        Correspondence &corr = data->obseravtions[i];
        int tNum = corr.value;

        Affine3DQ T;
        T.shift.loadFrom(&in[    MODCostFunctionData::MOVE_MODEL * tNum + MODCostFunctionData::OMNIDIR_POWER]);
        T.rotor.loadFrom(&in[3 + MODCostFunctionData::MOVE_MODEL * tNum + MODCostFunctionData::OMNIDIR_POWER]);
        T.shift.normalise();
        T.rotor.normalise();

        Ray3d r1(model.reverse(corr.start), Vector3dd::Zero());
        Ray3d r2(model.reverse(corr.end), Vector3dd::Zero());
        r2.a = T.rotor * r2.a;
        r2.p = T.shift + r2.p;

        Vector3dd e = r1.intersectCoef(r2);
        double t = 2 * e.z() / (e.x() * e.y());
        out[i] = t;
    }
}

void MODCostFunctionNormalise::operator()(const double in[], double out[])
{
    if (data == NULL) {
        return;
    }

    memcpy(out, in, sizeof(double) * inputs);

    int affineOffset = MODCostFunctionData::OMNIDIR_POWER;

    for (size_t i = 0; i < data->transform.size(); i++)
    {
        Vector3dd shift;
        shift.loadFrom(&out[MODCostFunctionData::MOVE_MODEL * i + affineOffset]);
        shift.normalise();
        shift.storeTo (&out[MODCostFunctionData::MOVE_MODEL * i + affineOffset]);

        Quaternion rotor;
        rotor.loadFrom(&out[3 + MODCostFunctionData::MOVE_MODEL * i + affineOffset]);
        rotor.normalise();
        rotor.storeTo (&out[3 + MODCostFunctionData::MOVE_MODEL * i + affineOffset]);
    }


}

MODCostFunctionData::MODCostFunctionData()
{

}

double MODModel::getCost(const Correspondence &corr)
{
    if (context == NULL) {
        SYNC_PRINT(("MODModel():getCost Internal Error\m"));
    }

    OmnidirectionalProjection projecton = context->projection;
    projecton.mN = projectionN;
    Affine3DQ T = transform[corr.value];

    Ray3d r1(projecton.reverse(corr.start), Vector3dd::Zero());
    Ray3d r2(projecton.reverse(corr.end), Vector3dd::Zero());
    r2.a = T.rotor * r2.a;
    r2.p = T.shift + r2.p;

    Vector3dd e = r1.intersectCoef(r2);
    return 2 * e.z() / (e.x() * e.y());
}
