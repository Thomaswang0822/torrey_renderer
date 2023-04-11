#pragma once

#include "torrey.h"
#include "vector.h"

#ifndef RAY_H
#define RAY_H

// From RTOW, ray.h
// Both point3 and vec3 are Vector3 in our context
class ray {
    public:
        ray() {}
        ray(const Vector3& origin, const Vector3& direction)
            : orig(origin), dir(normalize(direction))
        {}
        // 3rd Constructor: use orig and target point
        ray(const Vector3& origin, const Vector3& target, bool dummy)
            : orig(origin)
        {
            this->dir = normalize(target - origin);
        }

        Vector3 origin() const  { return orig; }
        Vector3 direction() const { return dir; }
        int srcObj() const { return src; }
        void setSrc(int id) {src = id; }

        Vector3 at(double t) const {
            return orig + t*dir;
        }

    private:
        int src = -1;    // Debug: which sphere (surface) it originates from
        Vector3 orig;
        Vector3 dir;
};

#endif