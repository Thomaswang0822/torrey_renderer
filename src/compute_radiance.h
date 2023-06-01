#pragma once

#include "BVH_node.h"
#include "helper.h"

/**
 * @brief This module will define all the top-level functions
 * in the rendering process.
 * 
 */

using namespace std;


/**
 * @brief Given a diffuse surface (or diffuse component of Plastic), compute the contribution
 *   from all lights in the scene.
 * @note used BEFORE path tracing (hw_4_1)
 * 
 * @param refl Color of the diffuse material at shading point
 * @return Vector3 
 */
Vector3 BVH_DiffuseColor(Scene& scene, Hit_Record& rec, const Color& refl, 
                        BVH_node& root, const Shape* hitObj, pcg32_state& rng);

/**
 * @brief Compute the total radiance (also the RGB value) received from a ray.
 *   The BRDF and corresponding pdf will be chosen according to the material
 * @note used BEFORE MIS path tracing (hw_4_3)
 * 
 * @return Vector3 
 */
Vector3 BVH_PixelColor(Scene& scene, ray& localRay, BVH_node& root, 
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


#pragma region PATH_TRACING
// out_dir (w_o in formula), BRDF_value, pdf_BRDF, pdf_Light
using Sample = tuple<Vector3, Vector3, Real, Real>;

/**
 * @brief Recursive function that calculates the Radiance arrived at 
 *     THIS PARTICULAR ray.
 * 
 * @note This is for HW_4_3, which implements one-sample MIS
 * 
 * @return Vector3 
 */
Vector3 radiance(Scene& scene, ray& localRay, BVH_node& root, 
                        pcg32_state& rng, unsigned int recDepth=MAX_DEPTH);


/**
 * @brief Perform BRDF sampling according to material
 * 
 * @note since using out_dir to get pdf_Light is non-trivial,
 *   we isolate its calculation into alternative_light_pdf() below.
 *   Thus, the 4-th return value is always 0.0
 * 
 * @note When dot product check fails, BRDF_value is 0, and last 2 pdf don't matter.
 * @return Sample 
 */
Sample BRDF_sample_dir(Material& currMaterial, Hit_Record& rec, 
            pcg32_state& rng, const Vector3& in_dir);

/**
 * @brief Perform Light sampling
 * 
 * @note Since finding the pdf_BRDF given the out_dir is a trivial task, 
 *   it will be computed here and returned in the 4-th return value
 * 
 * @note light_BRDF = Kd * I / PI, need to look at material anyway
 * @note determisitic MIS update: need the flexibility to either
 *     sample a light
 *     OR
 *     use the given light (sampled by the caller)
 * @param given_id default to -1, which means we should sample in the function. 
 *     Otherwise we use this id and don't perform sample
 * 
 * @return Sample 
 */
Sample Light_sample_dir(Scene& scene, Hit_Record& rec, BVH_node& root,
            Material& currMaterial, pcg32_state& rng, const Vector3& in_dir,
            int given_id=-1);


/**
 * @brief For MIS BRDF sampling usage: there are some additional work after
 *   we obtain a direction from BRDF sampling
 * 
 * @return Real 
 */
Real alternative_light_pdf(ray& outRay, Scene& scene, BVH_node& root,  // determine the light
            const Vector3& shadingPos);


/**
 * @brief This is a rewrite of the recursive function radiance().
 *     The iterative version can better accomodate Russian Roulette termination.
 * 
 * @ref It is based on PBRT book chapter 14.5.4, and those helpers it called.
 * 
 * @implements Deterministic MIS, see Step 4 & 5
 * @implements Russian Roulette, see Step 6
 * 
 * @note With Deterministic MIS, it supports point light
 * 
 * @param recDepth since this a iterative function, this is the max recusion depth and will not change
 * @return Vector3 
 */
Vector3 radiance_iterative(Scene& scene, ray& localRay, BVH_node& root, 
        pcg32_state& rng, unsigned int recDepth=MAX_DEPTH);


/**
 * @brief For deterministic MIS usage: 
 *     Uniformly pick a light from the scene.
 *     If it's a point Light, do calculation and return;
 *     If it's an area Light, do both:
 *     - Light sampling: pick a point from light and calculate contribution
 *     - BRDF sampling: pick a dir and do intersection check. Accumulate contribution 
 *       ONLY IF we hit the SAME area light along this direction, because we care
 *       "oneLight_contribution"
 * 
 * @ref https://www.pbr-book.org/3ed-2018/Light_Transport_I_Surface_Reflection/Direct_Lighting#UniformSampleOneLight
 * @ref https://www.pbr-book.org/3ed-2018/Light_Transport_I_Surface_Reflection/Direct_Lighting#EstimateDirect
 * 
 * 
 * @return Vector3 
 */
Vector3 sample_oneLight_contribution(Scene& scene, Hit_Record& rec, 
        BVH_node& root, pcg32_state& rng,
        Material& mat, const Vector3& in_dir);


/**
 * @brief Convert 2 pdfs in dir(solid angle) measurement back to area measurement.
 *      A function call isn't necessary but make caller code clearer
 * 
 * @param dsq distance squared
 * @param abs_cos |<out_dir, shading_normal>|
 */
inline void pdf_dir2pos(Real& pdf_BRDF, Real& pdf_Light, Real& dsq, Real abs_cos) {
    pdf_BRDF *= abs_cos / dsq;
    pdf_Light *= abs_cos / dsq;
}


/**
 * @brief For point light usage: given its position, calculate its radiance, f(x) in the formula.
 * 
 * @return Vector3 
 */
Vector3 compute_f_ptLight(Material& mat, const Vector3& in_dir,
            const Vector3& light_pos, Hit_Record& rec);


inline bool closeToZero(Vector3 v) {
    return length_squared(v) < 1e-9;
}

inline bool closeToZero(Real x) {
    return abs(x) < 1e-9;
}

/**
 * @brief Russian Roulette uses the luminance of a color to decide
 *     termination. The approach in PBRT is too complex. I found this
 *     one-line formula instead:
 * 
 * @ref https://stackoverflow.com/a/596243
 * 
 * @param rgb 
 * @return double 
 */
inline double Luminance(Vector3& rgb) {
    return 0.2126*rgb.x + 0.7152*rgb.y + 0.0722*rgb.z;
}

#pragma endregion PATH_TRACING