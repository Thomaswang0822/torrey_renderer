#pragma once

/**
 * @file helper.h
 * @brief This file defines global helper functions operating 
 * on the Scene. 
 */
#include "Scene.h"


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
     * @brief Given a normalized vector, use it as up-Z vector
     *   and generate a orthonormal basis
     * 
     * @param up 
     * @return Basis 
     */
    static Basis orthonormal_basis(Vector3 up) {
        // pick a random vector
        Vector3 some(1.0, 0.0, 0.0);
        if (abs(up.x) > 0.9) {
            // may have numerical issue when doing cross product
            some = {0.0, 1.0, 0.0};
        }
        Vector3 v = normalize(cross(up, some));
        Vector3 u = cross(up, v);
        return Basis{u, v, up};
    }

    inline Vector3 local2world(const Vector3& pos) {
        return pos.x * u + pos.y * v_up + pos.z * w;
    }
};

/**
 * @brief Spherical triangle on a unit sphere centered at (0,0,0)
 * 
 */
struct SphTriangle {
    Vector3 A,B,C;  // vertices
    double area;
    double alpha, beta, gamma;  // internal angles
    double a,b,c;  // edge lengths

    SphTriangle(const Triangle* tri, Vector3 center) {
        // Find vertices
        A = normalize(tri->p0 - center);
        B = normalize(tri->p1 - center);
        C = normalize(tri->p2 - center);

        // Find edge lengths: they are represented by 
        // "the lines connecting the vertices 
        // to the center of the sphere."
        // B - Point(0,0,0) = B
        a = acos(dot(B, C));
        b = acos(dot(A, C));
        c = acos(dot(A, B));

        // Find internal angles
        tie(alpha, beta, gamma) = internal_angles(A, B, C);

        // Find area
        area = alpha + beta + gamma - c_PI;
    }

    /**
     * @brief Compute 3 internal angles of a spherical triangle given 3 vertex positions
     * 
     * @def ext_i = sign(det(p_i-1, p_i, p_i+1)) * 
     *     acos(normalize(p_i-1 cross p_i) <dot> normalize(p_i cross p_i+1))
     * 
     * @ref Eq (1.2), Chern, Albert, and Sadashige Ishida. "Area formula for spherical polygons 
     *     via prequantization." arXiv preprint arXiv:2303.14555 (2023).
     * 
     * @return tuple<Real, Real, Real> 
     */
    tuple<Real, Real, Real> internal_angles(const Vector3& p0, const Vector3& p1, const Vector3& p2) {
        Vector3 cr01, cr12, cr20;
        cr01 = normalize(cross(p0, p1));
        cr12 = normalize(cross(p1, p2));
        cr20 = normalize(cross(p2, p0));

        // sign(det[p0, p1, p2]) is used to determine the signed angles,
        // but we only need unsigned
        Real ext0, ext1, ext2;  // external angles
        ext0 = abs(acos(dot(cr20, cr01)));
        ext1 = abs(acos(dot(cr01, cr12)));
        ext2 = abs(acos(dot(cr12, cr20)));
        // return value
        return {c_PI-ext0, c_PI-ext1, c_PI-ext2};
    }

    // return the sampled position on the local unit sphere centered at (0,0,0)
    Vector3 local_sample(pcg32_state& rng) {
        Real r1 = next_pcg32_real<Real>(rng);
        Real r2 = next_pcg32_real<Real>(rng);
        // Use one random variable to select the new area
        Real area_hat = r1 * area;
        // Save the sine and cosine of angle phi
        Real s = sin(area_hat - alpha);
        Real t = cos(area_hat - alpha);
        // Compute the pair u,v that deterines beta_hat
        Real u = t - cos(alpha);
        Real v = s + sin(alpha) * cos(c);
        // Let q be the cosine of the new edge length b_hat
        Real q = ((v * t - u * s) * cos(alpha) - v) /   // numerator
            ((v * s + u * t) * sin(alpha));  // denominator
        // Compute the third vertex of the sub-triangle
        // [x | y] = normalize(x - <x, y>*y)
        Vector3 C_hat = q * A + sqrt(1.0 - q*q) * normalize(C - dot(C, A)*A);
        // Use the other random variable to select cos_theta
        Real z = 1.0 - r2 * (1.0 - dot(C_hat, B));
        // Construct the corresponding position on the sphere
        Vector3 P = z * B + sqrt(1.0 - z*z) * normalize(C_hat - dot(C_hat, B) * B);
        return P;
    }
};


/**
 * @brief Given a Trianlge pointer, uniformly sample a point on the surface.
 * 
 * @note support stratified sampling of 4 equally divided sub-Triangles (by
 *   connect mid point on each edge)
 * 
 * @param tri 
 * @param rng 
 * @param which_part if defulat -1, we sample the entire triangle. otherwise
 *   we pick the sub-Triangle according to:
 *   0,1,2 -> choose the Triangle overlapping vertex 0, 1, 2
 *   3 -> choose the center Triangle
 *   This int should be generated by another random number
 * @return Vector3 
 */
Vector3 Triangle_sample(const Triangle* tri, pcg32_state& rng, int which_part=-1);

/**
 * @brief Perform the sampling described in "Stratified Sampling of Spherical Triangles"
 * 
 * @ref https://dl.acm.org/doi/pdf/10.1145/218380.218500
 * 
 * @return Vector3 
 */
Vector3 SphTri_sample(Triangle* tri, pcg32_state& rng, Hit_Record& rec);


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
 * @param cos_theta_max the angle taking cosine between Vector(hit_point, sphere light center)
 *   and Vector(sphere light radius)
 * @param cp NORMALIZED vector from sphere center to shading point; used as 
 *   up vector for the "tilted" sphere cap region.
 * @return Vector3 
 */
Vector3 Sphere_sample_cone(const Sphere* sph, pcg32_state& rng, 
                const Real& cos_theta_max, const Vector3& cp);



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


/**
 * @brief cosine hemisphere sampling, transformed to the world space 
 * 
 * @param rng 
 * @param basis the orthonormal basis at hit point, with hit normal as up vector
 * @return Vector3 
 */
Vector3 dir_cos_sample(pcg32_state& rng, Basis& basis);

/**
 * @brief Phong sampling around the mirror ray direction r
 * 
 * @param rng 
 * @param basis 
 * @param alpha Phong material exponent
 * @return Vector3 
 */
Vector3 dir_Phong_sample(pcg32_state& rng, Basis& basis, Real alpha);


Vector3 dir_GGX_sample(pcg32_state& rng, Basis& basis, Real exponent);