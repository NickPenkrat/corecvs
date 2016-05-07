#include "raytraceRenderer.h"

const double RaytraceableSphere::EPSILON = 0.000001;


RaytraceRenderer::RaytraceRenderer()
{

}

void RaytraceRenderer::trace(RayIntersection &intersection)
{
    // cout << "RaytraceRenderer::trace():" << intersection.ray << " " << intersection.depth << " " << intersection.weight << endl;
    bool hasInter = object->intersect(intersection);
    if (!hasInter || intersection.object == NULL) {
        // cout << "No intersection" << endl;
        return;
    }

    Raytraceable *obj = intersection.object;

    intersection.normal = obj->normal(intersection.ray.getPoint(intersection.t));

    if (obj->material == NULL)
        intersection.ownColor = obj->color;
    else {
        intersection.ownColor = TraceColor::Zero();
        obj->material->getColor(intersection, *this);
    }

    // SYNC_PRINT(("Has intersection!\n"));
}


void RaytraceRenderer::trace(RGB24Buffer *buffer)
{
    AbstractBuffer<TraceColor> energy(buffer->getSize());

    for (int i = 0; i < buffer->h; i++)
    {
        for (int j = 0; j < buffer->w; j++)
        {
            Vector2dd pixel(j, i);
            Ray3d ray = Ray3d(intrisics.reverse(pixel), Vector3dd::Zero());
            ray.normalise();

            RayIntersection intersection;
            intersection.ray = ray;
            intersection.weight = 1.0;
            intersection.depth = 0;
            trace(intersection);
            if (intersection.object != NULL) {
                energy.element(i, j) = intersection.ownColor;//TraceColor::FromDouble(intersection.normal);
            }
        }
    }

    for (int i = 0; i < buffer->h; i++)
    {
        for (int j = 0; j < buffer->w; j++)
        {
                buffer->element(i, j) = RGBColor::FromDouble(energy.element(i,j) / 2.0);
        }
    }
}


bool RaytraceableSphere::intersect(RayIntersection &intersection)
{
    Ray3d ray = intersection.ray;

//    SYNC_PRINT(("RaytraceableSphere::intersect([%lf %lf %lf] -> (%lf %lf %lf))\n", ray.p.x(), ray.p.y(), ray.p.z(), ray.a.x(), ray.a.y(), ray.a.z() ));
//    cout << "RaytraceableSphere::intersect():" << mSphere << endl;


    Vector3dd toCen  = mSphere.c  - ray.p;
    double toCen2 = toCen & toCen;
    double proj  = ray.a & toCen;
    double hdist  = (mSphere.r * mSphere.r) - toCen2 + proj * proj;
    double t2;

    if (hdist < 0) {
        return false;
    }

    hdist = sqrt (hdist);

    if (proj < 0) {
        if (hdist < CORE_ABS(proj) + EPSILON) {
            return false;
        }
    }

    if (hdist > CORE_ABS(proj))
    {
        intersection.t =  hdist + proj;
        t2 =  - hdist + proj;
    }
    else
    {
        intersection.t = proj - hdist;
        t2 = proj + hdist;
    }

    if (CORE_ABS(intersection.t) < EPSILON) intersection.t = t2;

    if (intersection.t > EPSILON) {
        intersection.object = this;
        return true;
    }
    return false;
}

Vector3dd RaytraceableSphere::normal(const Vector3dd &vector)
{
    return Vector3dd( (vector - mSphere.c) / mSphere.r);
}

bool RaytraceableSphere::inside(Vector3dd &point)
{
    Vector3dd tmp = mSphere.c - point;
    bool res;
    res = ((tmp & tmp) < (mSphere.r * mSphere.r));
    return res ^ !flag;
}

Raytraceable::~Raytraceable()
{

}

bool RaytraceableUnion::intersect(RayIntersection &intersection)
{
    RayIntersection best = intersection;
    best.t = std::numeric_limits<double>::max();

    for (Raytraceable *object: elements)
    {
        RayIntersection attempt = intersection;
        if (!object->intersect(attempt)) {
            continue;
        }
        if (attempt.t < best.t)
            best = attempt;
    }

    if (best.t == std::numeric_limits<double>::max()) {
        return false;
    }

    intersection = best;
    return true;
}

Vector3dd RaytraceableUnion::normal(const Vector3dd & /*vector*/)
{
    return Vector3dd::OrtX();
}

bool RaytraceableUnion::inside(Vector3dd &point)
{
    for (Raytraceable *object: elements)
    {
        if (object->inside(point))
            return true;
    }
    return false;
}



bool RaytraceablePlane::intersect(RayIntersection &intersection)
{
    intersection.object = NULL;
    bool hasIntersection = false;
    double t = mPlane.intersectWithP(intersection.ray, &hasIntersection);

    if (!hasIntersection)
        return false;

    if (t > 0.000001) {
        intersection.t = t;
        intersection.object = this;
        return true;
    }
    return false;
}

Vector3dd RaytraceablePlane::normal(const Vector3dd &vector)
{
    return mPlane.normal();
}

bool RaytraceablePlane::inside(Vector3dd &point)
{
    return mPlane.pointWeight(point);
}

void RaytraceableMaterial::getColor(RayIntersection &ray, RaytraceRenderer &renderer)
{

    ray.computeBaseRays();
    Vector3dd intersection = ray.getPoint();
    // cout << "RaytraceableMaterial::getColor() : " << intersection << endl;


    /* Recursive calls */

    if (ray.depth < 4 && ray.weight > 0.01)
    {
        if (reflCoef != 0.0) {
            RayIntersection reflected;
            reflected.ray = ray.reflection;
            reflected.ray.a.normalise();
            reflected.weight = ray.weight * reflCoef;
            reflected.depth = ray.depth + 1;
            reflected.ownColor = TraceColor::Zero();

            renderer.trace(reflected);
            ray.ownColor += reflCoef * reflected.ownColor;

            // cout << "Relection Ray brings:" << reflected.ownColor << endl;
        }

/*        if (refrCoef != 0.0) {
            RayIntersection refracted;
            refracted.ray = ray.reflection;
            refracted.weight = ray.weight * refrCoef;
            refracted.depth = ray.depth + 1;

            renderer.trace(refracted);
            ray.ownColor += refrCoef * refracted.ownColor;
        }*/
    }

    ray.ownColor += renderer.ambient;
    ray.ownColor += ambient;

    for (RaytraceablePointLight *light: renderer.lights)
    {
        cout << "Light сolor:" << light->color << " pos:" << light->position << endl;

        Vector3dd toLight = light->position  - intersection;

        //distance = -distance;

        RayIntersection lightRay;
        lightRay.depth  = 0;
        lightRay.weight = 1.0;

        lightRay.object = NULL;
        lightRay.ray.p = intersection;
        lightRay.ray.a = toLight;
        lightRay.ray.a.normalise();

        bool occluded = renderer.object->intersect(lightRay);
        if (occluded) {

            if (lightRay.t < (toLight.l2Metric() - 0.0001)) {
                SYNC_PRINT(("\nLight invisible %lf %lf\n", lightRay.t, (toLight.l2Metric() - 0.0001)));
                cout << "Origin :" << intersection << endl;
                cout << "Outcoming :" << toLight << endl;
                cout << "Intersection at" << lightRay.getPoint() << endl;
                cout << "With " << lightRay.object->name << endl;
                continue;
            }
        }

        // SYNC_PRINT(("Light visible: "));
        double attenuation = (1.0 / toLight.l2Metric());
        attenuation = 1.0;

        /* Diffuse part */
        double diffuseKoef = (ray.normal.normalised() & lightRay.ray.a.normalised());

        if (diffuseKoef < 0) diffuseKoef = 0.0;
        TraceColor diffusePart = light->color * attenuation * diffuseKoef * diffuse;


        /* Specular part */
        double specularKoef = pow(ray.reflection.a.normalised() & lightRay.ray.a.normalised(), specPower);

        if (specularKoef < 0) specularKoef = 0.0;
        TraceColor specularPart = light->color * attenuation * specularKoef * specular;

        // cout << "Light " << diffusePart  << "(" << diffuseKoef << ") ";
        // cout             << specularPart << "(" << specularKoef << ") " << endl;

        ray.ownColor += diffusePart;
        ray.ownColor += specularPart;

    }
}

void RayIntersection::computeBaseRays()
{
    Vector3dd intersection = getPoint();
    normal.normalised();

    reflection.p = intersection;
    Vector3dd dir = ray.a.normalised();
    reflection.a = dir - 2 * normal * (normal & dir);
    reflection.a.normalise();

    refraction.p = intersection;
    refraction.a = ray.a;
    refraction.a.normalise();
}

Vector3dd RayIntersection::getPoint()
{
    return ray.getPoint(t);
}

void RaytraceableChessMaterial::getColor(RayIntersection &ray, RaytraceRenderer &renderer)
{
    Vector3dd intersection = ray.getPoint();
    intersection = intersection / 10.0;

    bool b;
    int v = (int)intersection.x();
    b = (v > 0) ? (v % 2) : (v % 2);

    bool white = ((int)intersection.x() % 2) ^ ((int)intersection.y() % 2) ^ ((int)intersection.z() % 2);
    ray.ownColor = b ? TraceColor::Zero() : RGBColor::White().toDouble();
}
