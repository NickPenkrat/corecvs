#ifndef BSPTREE_H
#define BSPTREE_H

#include "line.h"
#include "polygons.h"
#include "axisAlignedBox.h"
#include "raytraceRenderer.h"
#include "conic.h"

namespace corecvs {

class NumPlaneFrame : public PlaneFrame {
public:
    int num;

    NumPlaneFrame(const PlaneFrame &frame, int num) :
        PlaneFrame(frame),
        num(num)
    {}
};

class NumTriangle3dd : public Triangle3dd {
public:
    int num;

    NumTriangle3dd(const Triangle3dd &triangle, int num) :
        Triangle3dd(triangle),
        num(num)
    {}

    NumPlaneFrame toNumPlaneFrame() const
    {
        return NumPlaneFrame(toPlaneFrame(), num);
    }
};

//template<class GeometryType, class CacheType, class BoundingType>
class BSPTreeNode {
public:
    typedef NumTriangle3dd GeometryType;
    typedef NumPlaneFrame CacheType;

    vector<GeometryType> submesh;
    vector<CacheType>    cached;


    BSPTreeNode *middle = NULL;
    BSPTreeNode *left = NULL;
    BSPTreeNode *right = NULL;

    Sphere3d bound;
    AxisAlignedBox3d box;
    Plane3d  plane;

    bool intersect(RayIntersection &intersection)
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

        for (CacheType &triangle : cached)
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
        BSPTreeNode *closer  = side ? left : right;
        BSPTreeNode *further = side ? right : left;


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

    void subdivide()
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
        for (const GeometryType &triangle : submesh)
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

        vector<GeometryType> m;
        vector<GeometryType> l;
        vector<GeometryType> r;
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
            middle = new BSPTreeNode;
            middle->submesh = m;
            middle->subdivide();
        }
        submesh.clear();

        if (!l.empty()) {
            left = new BSPTreeNode;
            left->submesh = l;
            left->subdivide();
        }

        if (!r.empty()) {
            right = new BSPTreeNode;
            right->submesh = r;
            right->subdivide();
        }
    }

    void cache()
    {
        cached.clear();
        for (const GeometryType &triangle : submesh)
        {
            cached.push_back(triangle.toNumPlaneFrame());
        }
        if (left  != NULL)  left->cache();
        if (right != NULL)  right->cache();
        if (middle != NULL) middle->cache();

    }

    int childCount()
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


    int triangleCount()
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


    void dumpToMesh(Mesh3D &mesh, int depth, bool plane, bool volume)
    {
        mesh.addIcoSphere(bound, 3);

        if (left)   left  ->dumpToMesh(mesh, depth + 1, plane, volume);
        if (right)  right ->dumpToMesh(mesh, depth + 1, plane, volume);
        if (middle) middle->dumpToMesh(mesh, depth + 1, plane, volume);
    }


    ~BSPTreeNode()
    {
        delete_safe(left);
        delete_safe(right);
        delete_safe(middle);
    }
};

} // namespace corecvs

#endif // BSPTREE_H

