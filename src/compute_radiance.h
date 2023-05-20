#pragma once

#include "BVH_node.h"
#include "helper.h"

/**
 * @brief This module will define all the top-level functions
 * in the rendering process.
 * 
 */

using namespace std;

// BVH_RaySceneHit() is actually BVH_node::hit
bool isVisible(const Vector3& shadingPt, Vector3& lightPos, Scene& scene, BVH_node& root);
Vector3 diffuse_radiance(Scene& scene, Hit_Record& rec, const Color& refl, 
                        BVH_node& root, const Shape* hitObj, pcg32_state& rng);
Vector3 radiance(Scene& scene, ray& localRay, BVH_node& root, 
                        pcg32_state& rng, unsigned int recDepth=MAX_DEPTH);

/**
 * @brief Compute the total SCALED contribution from an area-light TriangleMesh.
 * Sampling from mesh (alternative to looking at all Triangles) has been implemented.
 * It will be activated if sampleAll == true && mesh size >= maxSample
 * 
 * @note Each time we sample a point x_i from Triangle_i, the p(x) is just 1/area_i.
 *   Here we "expect" small contribution from each Triangle sum up to the total contribution
 *   of the mesh, so we don't take the average.
 * @note If we sample from mesh first, the probability of a Triangle_i being picked is
 *   its normalized area (area_i/mesh_surface_area). So overall, the p(x) = Prob(pick Triangle_i 
 *   from mesh) * Prob(pick x_i from Triangle_i) = area_i/mesh_surface_area * 1/area_i = 
 *   1/mesh_surface_area. In this sense, we "expect" each single sampling point to represent
 *   the total contribution from the mesh. Thus, we take the average at last. Quite counter-intuitive.
 * @note number of samples we really take is min(mesh.size(), maxSample). i.e. if the mesh only has
 *   16 Triangles, we take 16 samples instead of 64.
 * 
 * @param Kd diffuse surface reflectance at the hitting point
 * @param I area light radiance
 * @param sampleAll user defined flag of whether to sample lights
 *   (instead of looking at all lights in the mesh)
 * @param maxSample max number of light samples we pick from the mesh, default to 64
 * @param stratified whether use strafitied sampling. See helper::Triangle_sample()
 * 
 * @return Vector3 
 */
Vector3 meshLight_total_contribution(Scene& scene, Hit_Record& rec, BVH_node& root,
            int mesh_id, int shape_id,
            const Vector3& Kd, const Vector3& I,
            pcg32_state& rng, 
            bool sampleAll=true, int maxSample=64,
            bool stratified=false);



Vector3 sphereLight_contribution(Scene& scene, Hit_Record& rec, BVH_node& root,
            const Shape* lightObj, 
            const Vector3& Kd, const Vector3& I,
            pcg32_state& rng, 
            int ct=1);


using Sample = tuple<Vector3, Vector3, Real>;  // out_dir (w_o in formula), BRDF_value, pdf

Sample BRDF_sample(Material* currMaterial, Hit_Record& rec, 
            pcg32_state& rng, const Vector3& in_dir);

Sample Light_sample();

// For one-sample MIS usage
Real alternative_light_pdf(ray& outRay, Scene& scene, BVH_node& root,  // determine the light
            const Vector3& hitPos);
