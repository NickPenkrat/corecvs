#include "modCostFunction.h"

double MODCostFunction::cost(const Affine3DQ &T, const CameraProjection &projection, const Correspondence &corr)
{

    Ray3d r1(projection.reverse(corr.start), Vector3dd::Zero());
    Ray3d r2(projection.reverse(corr.end  ), Vector3dd::Zero());
    r2.a = T.rotor * r2.a;
    r2.p = T.shift + r2.p;

    Vector3dd e = r1.intersectCoef(r2);
    return 2 * e.z() / (e.x() + e.y());
}

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

        Affine3DQ T = MODCostFunctionData::load(&in[MODCostFunctionData::MOVE_MODEL * tNum + MODCostFunctionData::OMNIDIR_POWER]);
        T.shift.normalise();
        T.rotor.normalise();
        out[i] = cost(T, model, corr);
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
        int offset = MODCostFunctionData::MOVE_MODEL * i + affineOffset;
        Affine3DQ T = MODCostFunctionData::load(&out[offset]);
        T.shift.normalise();
        T.rotor.normalise();
        MODCostFunctionData::store(T, &out[offset]);
    }
}

MODCostFunctionData::MODCostFunctionData()
{

}

Affine3DQ MODCostFunctionData::load(const double *in)
{
    Affine3DQ toReturn;
    toReturn.shift.loadFrom(in    );
    toReturn.rotor.loadFrom(in + 3);
    return toReturn;
}

void MODCostFunctionData::store(const Affine3DQ &input, double *out)
{
    input.shift.storeTo(out    );
    input.rotor.storeTo(out + 3);
}

double MODModel::getCost(const Correspondence &corr)
{
    if (context == NULL) {
        SYNC_PRINT(("MODModel():getCost Internal Error\n"));
    }
    OmnidirectionalProjection projection = context->projection;
    projection.mN = projectionN;
    Affine3DQ T = transform[corr.value];
    return MODCostFunction::cost(T, projection, corr);
}
