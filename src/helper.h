#pragma once

/**
 * @file helper.h
 * @brief This file defines global helper functions operating 
 * on the Scene. 
 */
#include "Scene.h"


#include <filesystem>
#include <variant>
#include <vector>


void checkRaySphereHit(ray localRay,
                       const Sphere* sph,
                       Hit_Record& rec,
                       Shape*& hitObj);


void checkRayTriHit(ray localRay,
                    const Triangle* tri,
                    Hit_Record& rec,
                    Shape*& hitObj);

void checkRayShapeHit(ray localRay,
                    Shape& curr_shape,
                    Hit_Record& rec,
                    Shape*& hitObj);

void checkRaySceneHit(ray localRay,
                      Scene& scene,
                      Hit_Record& rec,
                      Shape*& hitObj);

struct Basis {
    Vector3 u, v_up, w;

    /**
     * @brief Given a normalized vector, use it as up vector
     *   and generate a orthonormal basis
     * 
     * @param v_up 
     * @return Basis 
     */
    static Basis orthonormal_basis(Vector3 v_up) {
        // pick a random vector
        Vector3 some(1.0, 0.0, 0.0);
        if (abs(dot(v_up, some)) < 1e-6) {
            // may have numerical issue when doing cross product
            some = {0.0, 0.0, 1.0};
        }
        Vector3 u = normalize(cross(v_up, some));
        Vector3 w = cross(u, v_up);
        return Basis{u, v_up, w};
    }
};

/**
 * @brief Spherical triangle on a unit sphere
 * 
 */
struct SphTriangle {
    Vector3 A,B,C;  // vertices
    double area;
    double alpha, beta, gamma;  // internal angles
    double a,b,c;  // edge lengths

    SphTriangle(const Triangle* tri, Vector3 center) {
        Vector3 oA = normalize(tri->p0 - center);
        Vector3 oB = normalize(tri->p0 - center);
        Vector3 oC = normalize(tri->p0 - center);
        // Find vertices
        A = center + oA;
        B = center + oB;
        C = center + oC;

        // Find edge lengths: they are represented by 
        // "by the lines connecting the vertices 
        // to the center of the sphere."
        a = acos(dot(oB, oC));
        b = acos(dot(oA, oC));
        c = acos(dot(oA, oB));

        // TODO: Find internal angles

        // Find area
        area = alpha + beta + gamma - c_PI;
    }
};


/**
 * @brief Given a Trianlge pointer, uniformly sample a point on the surface
 * 
 * @param tri 
 * @param rng 
 * @return Vector3 
 */
Vector3 Triangle_sample(const Triangle* tri, pcg32_state& rng);


/**
 * @brief Given a Sphere pointer, uniformly sample a point on the entire surface.
 * 
 * @note support stratified sampling of equally spaced azumith angle. i.e. an 
 *   orange sphere will be divided into `ct` slices and we sample one point on 
 *   each. This function will be called `ct` times.
 * 
 * @param sph 
 * @param rng
 * @param idx the index of the "orange slice"
 * @param ct stratified sample count, default to 1 (no stratified sample)
 * @return Vector3 
 */
Vector3 Sphere_sample(const Sphere* sph, pcg32_state& rng, int idx=0, int ct=1);

/**
 * @brief Given a Sphere pointer, sample a point using cone sampling
 * 
 * @note unlike PBRT and RTRYL, which uses the thin angle between 
 *   Vector(hit_point, sphere light center) and 
 *   Vector(hit_point, tangent point on the sphere), my theta_max is described above.
 *   This difference arises from that they use "sampling direction at shading point",
 *   we use "sampling a point from the area light", the Sphere.
 * 
 * @param sph 
 * @param rng 
 * @param theta_max the angle between Vector(hit_point, sphere light center)
 *   and Vector(sphere light radius)
 * @param cp NORMALIZED vector from sphere center to shading point; used as 
 *   up vector for the "tilted" sphere cap region.
 * @return Vector3 
 */
Vector3 Sphere_sample_cone(const Sphere* sph, pcg32_state& rng, 
                const Real& theta_max, const Vector3& cp);



/**
 * @brief Given a area-lighted *primitive* object, return the UNSCALED RGB light
 * contribution of it.
 * 
 * @note for a TriangleMesh area light, each Trianlge is unpacked outside the function,
 * so does the average over N (number of Triangles in the mesh)
 * @note visibility is also determined outside
 * 
 * @param lightPos sample position
 * @param Kd diffuse surface reflectance at the hitting point
 * @param I area light radiance
 * @param nx normal at light source (flip toward hitting point for Triangle)
 * @return Vector3 
 */
Vector3 areaLight_contribution(const Shape* lightObj, Hit_Record& rec, 
            const Vector3& lightPos, const Vector3& Kd, 
            const Vector3& I, const Vector3& nx);
