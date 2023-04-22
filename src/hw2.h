#pragma once

#include "image.h"
#include "ray.h"
#include "Scene.h"
#include "pcg.h"

Image3 hw_2_1(const std::vector<std::string> &params);
Image3 hw_2_2(const std::vector<std::string> &params);
Image3 hw_2_3(const std::vector<std::string> &params);
Image3 hw_2_4(const std::vector<std::string> &params);
Image3 hw_2_5(const std::vector<std::string> &params);

/* 
class Triangle {
public:
    Vector3 p0, p1, p2;  // vertices position
    Vector3 e1, e2;  // p1-p0, p2-p0; not normalized

    Triangle(Vector3 pos0, Vector3 pos1, Vector3 pos2) :
        p0(pos0), p1(pos1), p2(pos2),
        e1(pos1 - pos0), e2(pos2 - pos0) {}
}; */

inline bool RayIntersectsTriangle(ray localRay, 
                           Triangle tri,
                           Real& dist,
                           Vector3& outIntersectionPoint,
                           Vector3& baryC)
{
    const float EPSILON = 1e-7;

    Vector3 h, s, q;
    Real a, f, u, v;
    h = cross(localRay.direction(), tri.e2);
    a = dot(tri.e1, h);

    if (a > -EPSILON && a < EPSILON) {
        return false;    // This ray is parallel to this triangle.
    }
    f = 1.0 / a;
    s = localRay.origin() - tri.p0;
    u = f * dot(s, h);

    if (u < 0.0 || u > 1.0) {
        return false;   // hit point not in triangle
    }

    q = cross(s, tri.e1);
    v = f * dot(localRay.direction(), q);

    if (v < 0.0 || u + v > 1.0) {
        return false;   // hit point not in triangle
    }

    // At this stage we can compute t to find out where the intersection point is on the line.
    Real t = f * dot(tri.e2, q);

    if (t > EPSILON) // ray intersection
    {
        // only update storage variables before returning true
        dist = t;
        outIntersectionPoint = localRay.at(t);
        baryC = {1.0-u-v, u, v};
        return true;
    }
    else {// This means that there is a line intersection but not a ray intersection.
        return false;
    }
};


inline bool RayIntersectsAny_Naive(ray localRay, 
                           std::vector<Triangle> tris,
                           Real& dist,  // should be inf when passed in
                           Vector3& outIntersectionPoint,
                           Vector3& baryC)
{
    Real currT;
    Vector3 currHitPos, currBaryC;
    // Brute-force: look at each triangle and update only if (hit && closer)
    for (Triangle tri : tris) {
        if (RayIntersectsTriangle(localRay, tri, currT, currHitPos, currBaryC)) {
            if (currT < dist) {
                dist = currT;
                outIntersectionPoint = Vector3(currHitPos);
                baryC = Vector3(currBaryC);
            }
        }
    }
    return bool(dist < infinity<Real>());
}
