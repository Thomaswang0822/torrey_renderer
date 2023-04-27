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

inline ray mirror_ray(ray& rayIn, Vector3 outNormal, Vector3& hitPt) {
    if (dot(rayIn.direction(), outNormal) > 0.0) {
        
        return mirror_ray(rayIn, -outNormal, hitPt);
    }
    Vector3 outDir = normalize(rayIn.direction() - 2*dot(rayIn.direction(),outNormal)*outNormal);
    if (dot(outDir, outNormal) < 0.0) {
        throw std::runtime_error("OUT direction is incorrect");
    }
    return ray(
        hitPt,      // origin
        outDir  // reflected dir
    );
}

/**
 * @brief Given required info, generate a refract ray
 * 
 * @note: this function works for both ray inside & outside sphere and
 * does not distinguish them.
 * 
 * @param rayIn 
 * @param outNormal 
 * @param hitPt 
 * @param eta_ratio: eta / eta_prime, which are refractive indices
 * 
 * @return ray 
 */
inline ray refract_ray(ray& rayIn, Vector3 outNormal, Vector3& hitPt, double eta_ratio) {
    double cos_theta = dot(-rayIn.direction(), outNormal);
    Vector3 r_out_perp = eta_ratio * (rayIn.direction() + cos_theta * outNormal);
    Vector3 r_out_parallel = -sqrt(fabs(1.0 - length_squared(r_out_perp))) * outNormal;
    return ray(
        hitPt,
        r_out_perp + r_out_parallel     // constructor will take care of normalize()
    );
}

// Use Schlick's approximation for reflectance.
inline double reflectance(double cosine, double eta_ratio) {
    auto r0 = (1-eta_ratio) / (1+eta_ratio);
    r0 = r0*r0;
    return r0 + (1-r0)*pow((1 - cosine),5);
}

#endif