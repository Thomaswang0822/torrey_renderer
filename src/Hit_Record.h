#pragma once

#include "all_utils.h"
#include "ray.h"

struct Hit_Record {
    Vector3 pos;
    Vector3 normal;
    int mat_id;
    Real dist;  // hit distance
    double u, v;  // uv coordinate of hit point local to hit obj
    bool front_face;

    // Default constructor: set hitDist = infinity
    Hit_Record() : 
    dist(infinity<Real>()) {}

    inline void set_face_normal(const ray& r, const Vector3& outward_normal) {
        front_face = dot(r.dir, outward_normal) < 0;
        normal = front_face ? outward_normal :-outward_normal;
    }
};

