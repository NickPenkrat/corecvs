#include "raytraceObjects.h"

const double RaytraceableSphere::EPSILON = 0.000001;


/**
   We now can use Sphere3d method for this.
   This method need to be corrected
 **/
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

void RaytraceableSphere::normal(RayIntersection &intersection)
{
    intersection.normal = Vector3dd( (intersection.getPoint() - mSphere.c) / mSphere.r);
}

bool RaytraceableSphere::inside(Vector3dd &point)
{
    Vector3dd tmp = mSphere.c - point;
    bool res;
    res = ((tmp & tmp) < (mSphere.r * mSphere.r));
    return res ^ !flag;
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

void RaytraceableUnion::normal(RayIntersection & /*intersection*/)
{
    return;
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

void RaytraceablePlane::normal(RayIntersection &intersection)
{
    intersection.normal = mPlane.normal();
}

bool RaytraceablePlane::inside(Vector3dd &point)
{
    return mPlane.pointWeight(point);
}

bool RaytraceableTriangle::intersect(RayIntersection &intersection)
{
    intersection.object = NULL;
    double t = 0;

    if (!mTriangle.intersectWithP(intersection.ray, t))
    {
        return false;
    }
    if (t > 0.000001) {
        intersection.t = t;
        intersection.object = this;
        return true;
    }
    return false;
}

void RaytraceableTriangle::normal(RayIntersection &intersection)
{
    intersection.normal = mTriangle.getNormal();
}

bool RaytraceableTriangle::inside(Vector3dd & /*point*/)
{
    return false;
}

RaytraceableMesh::RaytraceableMesh(Mesh3DDecorated *mesh) :
    mMesh(mesh)
{


}

bool RaytraceableMesh::intersect(RayIntersection &intersection)
{

    RayIntersection best = intersection;
    best.t = std::numeric_limits<double>::max();
    // SYNC_PRINT(("RaytraceableMesh::intersect(): entered\n"));

    for (size_t i = 0; i < mMesh->faces.size(); i++)
    {
        Triangle3dd triangle = mMesh->getFaceAsTrinagle(i);

        double t = 0;
        if (!triangle.intersectWithP(intersection.ray, t))
        {
            continue;
        }

        if (t > 0.000001 && t < best.t)
        {
            best.t = t;
            best.normal = triangle.getNormal();
            best.object = this;
        }
    }

    if (best.t == std::numeric_limits<double>::max()) {
        // SYNC_PRINT(("RaytraceableMesh::intersect(): passed\n"));

        return false;
    }

    intersection = best;
    return true;

}

void RaytraceableMesh::normal(RayIntersection & /*intersection*/)
{
    return;
}

bool RaytraceableMesh::inside(Vector3dd & /*point*/)
{
    return false;
}




bool RaytraceableTransform::intersect(RayIntersection &intersection)
{
    RayIntersection trans = intersection;
    trans.ray.p = mMatrixInv * intersection.ray.p;
    trans.ray.a = mMatrixInv * intersection.ray.a;
    double len = trans.ray.a.l2Metric();
    trans.ray.a /= len;

    //trans.ray.a.normalise();

    if (mObject->intersect(trans)) {
        intersection.object = this;
        intersection.t = trans.t / len;
        //intersection.normal = mMatrix.inverted() * trans.normal;
        return true;
    }
    return false;
}

void RaytraceableTransform::normal(RayIntersection &intersection)
{
    //normal = Vector3dd::OrtZ();
    RayIntersection trans = intersection;
    double scale1 = trans.ray.a.l2Metric();
    trans.ray.transform(mMatrixInv);
    double scale2 = trans.ray.a.l2Metric();
    trans.t = trans.t / scale2 * scale1;

    mObject->normal(trans);
    intersection.normal = mMatrix * intersection.normal;
    intersection.normal.normalise();
}

bool RaytraceableTransform::inside(Vector3dd &point)
{
    Vector3dd p = mMatrix * point;
    return mObject->inside(p);
}

bool RaytraceableOptiMesh::TreeNode::intersect(RayIntersection &intersection)
{
    intersection.object = NULL;
    double t = 0;

    double d1,d2;
    if (!box.intersectWith(intersection.ray, d1, d2))
        return false;

    if (d2 <= 0)
        return false;
    /*if (!box.intersectWith(intersection.ray, d1, d2))
        return false;*/

    RayIntersection best = intersection;
    best.t = std::numeric_limits<double>::max();

    for (NumPlaneFrame &triangle : cached)
    {
        double u, v;
        if (!triangle.intersectWithP(intersection.ray, t, u, v))
            continue;

        if (t > 0.000001 && t < best.t) {
            best.t = t;
            best.normal = triangle.getNormal();
            best.uvCoord = Vector2dd(u, v);
            best.payload = triangle.num;
        }
    }

    if (middle != NULL) {
        bool result = middle->intersect(intersection);
        if (result) {
            if (intersection.t > 0.000001 && intersection.t < best.t) {
                best = intersection;
            }
        }
    }

    bool side = plane.pointWeight(intersection.ray.p) > 0;
    TreeNode *closer  = side ? left : right;
    TreeNode *further = side ? right : left;


    if (closer != NULL) {
        bool result = closer->intersect(intersection);
        if (result) {
            if (intersection.t > 0.000001 && intersection.t < best.t) {
                best = intersection;
            }
        }
    }


    if (further != NULL) {
        bool result = further->intersect(intersection);
        if (result) {
            if (intersection.t > 0.000001 && intersection.t < best.t) {
                best = intersection;
            }
        }
    }

    if (best.t != std::numeric_limits<double>::max()) {
        intersection = best;
        return true;
    }
    return false;
}

void RaytraceableOptiMesh::TreeNode::subdivide()
{
    //SYNC_PRINT(("RaytraceableOptiMesh::TreeNode::subdivide() : %u nodes", middle.size()));
    if (submesh.size() == 0)
    {
        SYNC_PRINT(("RaytraceableOptiMesh::TreeNode::subdivide() : empty node\n"));
        return;
    }

    Vector3dd minP = Vector3dd(numeric_limits<double>::max());
    Vector3dd maxP = Vector3dd(numeric_limits<double>::lowest());

    EllipticalApproximation3d approx;
    for (const NumTriangle3dd &triangle : submesh)
    {
        approx.addPoint(triangle.p1());
        approx.addPoint(triangle.p2());
        approx.addPoint(triangle.p3());

        for (int i = 0; i < Triangle3dd::SIZE; i++)
        {
            for (int j = 0; j < Vector3dd::LENGTH; j++)
            {
                if (minP[j] > triangle.p[i][j]) minP[j] = triangle.p[i][j];
                if (maxP[j] < triangle.p[i][j]) maxP[j] = triangle.p[i][j];
            }
        }
    }

    Vector3dd center = approx.getCenter();
    approx.getEllipseParameters();

    Vector3dd normal = approx.mAxes[0];
    plane = Plane3d::FromNormalAndPoint(normal, center);

    double radius = 0;
    for (const NumTriangle3dd &triangle : submesh)
    {
        for (int p = 0; p < 3; p++)
        {
            double d = (triangle.p[p] - center).l2Metric();
            if (radius < d)
                radius = d;
        }
    }
    bound = Sphere3d(center, radius + 0.000001);
    minP -= Vector3dd(0.000001);
    maxP += Vector3dd(0.000001);
    box = AxisAlignedBox3d(minP, maxP);

    if (submesh.size() <= 3)
        return;

    vector<NumTriangle3dd> m;
    vector<NumTriangle3dd> l;
    vector<NumTriangle3dd> r;
    for (const NumTriangle3dd &triangle : submesh)
    {
        bool b1 = (plane.pointWeight(triangle.p1()) > 0);
        bool b2 = (plane.pointWeight(triangle.p2()) > 0);
        bool b3 = (plane.pointWeight(triangle.p3()) > 0);

        if (b1 && b2 && b3) {
            l.push_back(triangle);
            continue;
        }

        if (!b1 && !b2 && !b3) {
            r.push_back(triangle);
            continue;
        }

        m.push_back(triangle);
    }

    /* Check if there was a subdivison actually */
    if (m.size() == submesh.size())
        return;
    if (l.size() == submesh.size())
        return;
    if (r.size() == submesh.size())
        return;


    //submesh = m;
    delete_safe(middle);
    delete_safe(left);
    delete_safe(right);

    //SYNC_PRINT(("RaytraceableOptiMesh::TreeNode::subdivide() : groups %u (%u | %u) nodes\n", m.size(), l.size(), r.size()));

    if (!m.empty()) {
        middle = new TreeNode;
        middle->submesh = m;
        middle->subdivide();
    }
    submesh.clear();

    if (!l.empty()) {
        left = new TreeNode;
        left->submesh = l;
        left->subdivide();
    }

    if (!r.empty()) {
        right = new TreeNode;
        right->submesh = r;
        right->subdivide();
    }
}

void RaytraceableOptiMesh::TreeNode::cache()
{
    cached.clear();
    for (const NumTriangle3dd &triangle : submesh)
    {
        cached.push_back(triangle.toNumPlaneFrame());
    }
    if (left  != NULL)  left->cache();
    if (right != NULL) right->cache();
    if (middle != NULL) middle->cache();

}

int RaytraceableOptiMesh::TreeNode::childCount()
{
    int sum = 1;
    if (left) {
        sum += left->childCount();
    }
    if (right) {
        sum += right->childCount();
    }
    if (middle) {
        sum += middle->childCount();
    }
    return sum;
}

int RaytraceableOptiMesh::TreeNode::triangleCount()
{
    int sum = (int)submesh.size();
    if (left) {
        sum += left->triangleCount();
    }
    if (right) {
        sum += right->triangleCount();
    }
    if (middle) {
        sum += middle->triangleCount();
    }
    return sum;
}

void RaytraceableOptiMesh::TreeNode::dumpToMesh(Mesh3D &mesh, int depth, bool plane, bool volume)
{
    mesh.addIcoSphere(bound, 3);

    if (left)   left  ->dumpToMesh(mesh, depth + 1, plane, volume);
    if (right)  right ->dumpToMesh(mesh, depth + 1, plane, volume);
    if (middle) middle->dumpToMesh(mesh, depth + 1, plane, volume);
}

void RaytraceableOptiMesh::optimize()
{
    delete_safe(opt);
    opt = new TreeNode();
    for (size_t i = 0; i < mMesh->faces.size(); i++)
    {
        NumTriangle3dd triangle(mMesh->getFaceAsTrinagle(i), (int)i);
        opt->submesh.push_back(triangle);
    }
    opt->subdivide();
    opt->cache();
}

void RaytraceableOptiMesh::dumpToMesh(Mesh3D &mesh, bool plane, bool volume)
{
    if (opt == NULL) {
        SYNC_PRINT(("RaytraceableOptiMesh: not optimized"));
        return;
    }

    opt->dumpToMesh(mesh, 0, plane, volume);

}

bool RaytraceableOptiMesh::intersect(RayIntersection &intersection)
{
    intersection.object = NULL;

    if (opt == NULL) {
        // SYNC_PRINT(("RaytraceableOptiMesh::intersect(): not optimized"));
        return false;
    }

    if (opt->intersect(intersection))
    {
        // SYNC_PRINT(("RaytraceableOptiMesh::intersect(): We have intersection (%lf)\n", intersection.t));
        intersection.object = this;
        return true;
    }
    return false;
}

void RaytraceableOptiMesh::normal(RayIntersection &intersection)
{
//    SYNC_PRINT(("RaytraceableOptiMesh::normal(%d, %d)\n", intersection.payload, mMesh->hasNormals));

    if (intersection.payload != -1 )
    {
        //Vector3d32 face = mMesh->faces[intersection.payload];
        Vector3d32 normalId  = mMesh->normalId[intersection.payload];
        Vector3d32 textureId = mMesh->texId   [intersection.payload];

        double u = intersection.uvCoord.x();
        double v = intersection.uvCoord.y();

        if(mMesh->hasNormals) {
            Vector3dd n =
                    mMesh->normalCoords[normalId.x()] * (1 - u - v) +
                    mMesh->normalCoords[normalId.y()] * u +
                    mMesh->normalCoords[normalId.z()] * v;
            intersection.normal = n.normalised();
        }

        if (mMesh->hasTexCoords) {
            Vector2dd tex =
                    mMesh->textureCoords[textureId.x()] * (1 - u - v) +
                    mMesh->textureCoords[textureId.y()] * u +
                    mMesh->textureCoords[textureId.z()] * v;
            intersection.texCoord = tex;

        }
//        SYNC_PRINT(("Augmented Normal\n"));
    }
    //intersection.texCoord = mMesh->textureCoords[face.x()];
}

#if 0
bool RaytraceableCylinder::intersect(RayIntersection &intersection)
{
    intersection.object = NULL;

    Plane3d bottom = mCylinder.getBottomPlane();
    Plane3d top    = mCylinder.getTopPlane();

    Ray3d axis = mCylinder.getAxis();




    Ray3d projected = mCylinder.getBottomPlane().projectRayTo(intersection.ray);
    Sphere3d tmp(mCylinder);

    double t[4] = { std::numeric_limits<double>::lowest() };
    bool bc = tmp.intersectWith(projected, t[0], t[1]);

    t[0] /= projected.a.l2Metric();
    t[1] /= projected.a.l2Metric();

    bool b1 = true;
    t[2] = bottom.intersectWithP(intersection.ray, &b1);
    if (!b1) t[2] = std::numeric_limits<double>::lowest();

    bool b2 = true;
    t[3] = top.intersectWithP(intersection.ray, &b2);
    if (!b2) t[3] = std::numeric_limits<double>::lowest();

    double best = std::numeric_limits<double>::max();
    int n = -1;

    for (size_t i = 0; i < CORE_COUNT_OF(t); i++ )
    {
        if (t[i] > 0 && t[i] < best) {
            best = t[i];
            n = i;
        }
    }

    if (n == -1)
    {
        return false;
    }

    intersection.object = this;
    intersection.t = best;

    if (n == 0 || n == 1) {
        intersection.normal = mCylinder.normal;
    } else {
        Vector3dd point = intersection.getPoint();
        intersection.normal = point - axis.projection(point);
        intersection.normal.normalise();
    }
    return true;
}

bool RaytraceableCylinder::intersect(RayIntersection &intersection)
{
    intersection.object = NULL;

    Ray3d ray  = intersection.ray.normalised();
    Ray3d axis = mCylinder.getAxis().normalised();

//    cout << "RaytraceableCylinder::intersect():  ray:" << ray  << endl;
//    cout << "RaytraceableCylinder::intersect(): axis:" << axis << endl;

    Vector3dd coef = ray.intersectCoef(axis);

//    cout << "RaytraceableCylinder::intersect(): coef:" << coef << endl;

    if (fabs(coef.z()) > mCylinder.r)
        return false;

    double ca = (ray.a & axis.a);
    double sa = sqrt(1.0 - (ca * ca));

//    cout << "RaytraceableCylinder::intersect(): ca, sa:" << ca << " " << sa << endl;

    double l1 = coef.z() * ca;
    double l2 = coef.z() * sa;
    double dt = sqrt(mCylinder.r * mCylinder.r - l2 * l2);

    double h1 = coef.y() - l2 * mCylinder.r;
    double h2 = coef.y() + l2 * mCylinder.r;

    if        (coef.x() - dt  > 0.0001 && h1 < mCylinder.height && h1 > 0 ) {
        intersection.t = coef.x() - dt;
        intersection.object = this;
        return true;
    } else if (coef.x() + dt  > 0.0001 && h2 < mCylinder.height && h2 > 0 )  {
        intersection.t = coef.x() + dt;
        intersection.object = this;
        return true;
    }
    return false;
}
#endif


bool RaytraceableCylinder::intersect(RayIntersection &intersection)
{
    static const double CYL_EPSILON  = 0.00001;
    intersection.object = NULL;

    Ray3d &ray = intersection.ray;
    Vector3dd shift = ray.p - p;
    Ray3d cylFrame;
    cylFrame.a.x() = ray.a & e1;
    cylFrame.a.y() = ray.a & e2;
    cylFrame.a.z() = ray.a &  n;

    cylFrame.p.x() = shift & e1;
    cylFrame.p.y() = shift & e2;
    cylFrame.p.z() = shift &  n;



    double b = - (cylFrame.p.xy() & cylFrame.a.xy()) / (cylFrame.a.xy() & cylFrame.a.xy());
    Vector2dd pos = cylFrame.p.xy() + b * cylFrame.a.xy();
    double x = !pos;

    if (x > r) {
        return false;
    }

    double db = sqrt(r * r - x * x);

    double t1 = b - db;
    double t2 = b + db;

    /*
    double t1 = -1;
    double t2 = -1;
    */

    double len1 = (cylFrame.p.z() + t1 * cylFrame.a.z()) / h;
    double len2 = (cylFrame.p.z() + t2 * cylFrame.a.z()) / h;

    Plane3d bottom = Plane3d::FromNormalAndPoint(-n, p        );
    Plane3d top    = Plane3d::FromNormalAndPoint( n, p + n * h);


/*    cout << "RaytraceableCylinder::intersect():  ray:" << cylFrame  << endl;
    cout << "RaytraceableCylinder::intersect():    b:" << b << endl;
    cout << "RaytraceableCylinder::intersect():  pos:" << pos << endl;
    cout << "RaytraceableCylinder::intersect():   db:" << db << endl;
    cout << "RaytraceableCylinder::intersect():    t:" << t1 << " " << t2 << endl;

    cout << "RaytraceableCylinder::intersect():  len:" << len1 << " " << len2 << endl;
*/


    //if ( cylFrame.a.z() > CYL_EPSILON || cylFrame.a.z() < -CYL_EPSILON)
    {
        bool ok1;
        bool ok2;
        double tp1 = bottom.intersectWithP(ray, &ok1);
        double tp2 = top   .intersectWithP(ray, &ok2);
        Vector3dd pp1 = ray.getPoint(tp1) - p ;
        Vector3dd pp2 = ray.getPoint(tp2) - p - n * h;




        if      (len1 < 0.0)
        {
            if ( !ok1 || (pp1 & pp1) >= r * r ) {
                t1 = -1;
            } else {
                t1 = tp1;
            }
        }
        else if (len1 > 1.0)
        {
            if ( !ok2 || (pp2 & pp2) >= r * r ) {
                t1 = -1;
            } else {
                t1 = tp2;
            }
        }

        if      (len2 < 0.0)
        {
            if ( !ok1 || (pp1 & pp1) >= r * r ) {
                t2 = -1;
            } else {
                t2 = tp1;
            }
        }
        else if ( len2 > 1.0)
        {
            if ( !ok2 || (pp2 & pp2) >= r * r ) {
                t2 = -1;
            } else {
                t2 = tp2;
            }
        }
    }

    if (t1 > t2) std::swap(t1, t2);
/*

    double a = cylFrame.a.xy().sumAllElementsSq();
    double b = cylFrame.p.xy() & cylFrame.a.xy();
    double c = cylFrame.p.xy().sumAllElementsSq() - r * r * r *r;

    double d = b * b - a * c;

    if (d <= 0.0)
        return false;
    d = sqrt(d);

    double t1 = (-b - d) / a;
    double t2 = (-b + d) / a;

    double len1 = (cylFrame.a.z() + t1*cylFrame.p.z()) / h;
    double len2 = (cylFrame.a.z() + t2*cylFrame.p.z()) / h;
*/

    if ( t1 > CYL_EPSILON)
    {
       intersection.object = this;
       intersection.t      = t1;
       return true;
    }

    if ( t2 > CYL_EPSILON)
    {
       intersection.object = this;
       intersection.t      = t2;
       return true;
    }
    return false;

}

void RaytraceableCylinder::normal(RayIntersection &intersection)
{
    static const double CYL_EPSILON  = 0.00001;

    Vector3dd pos = intersection.getPoint();
    double t = ((pos - p) & n);

    if ( t <  CYL_EPSILON )
    {
        intersection.normal = - n;
        return;
    }
    if ( t > h - CYL_EPSILON) {
        intersection.normal = n;
        return;
    }

    intersection.normal  = (pos - p - n * t).normalised();
    return;
}

bool RaytraceableCylinder::inside(Vector3dd &point)
{
    Ray3d axis(n, p);
    double originProj = axis.projectionP(point);

    if (originProj < 0 || originProj > h)
        return flag;

    double d = axis.distanceTo(point);
    if (d > r)
    {
        return flag;
    }

    return !flag;

}
