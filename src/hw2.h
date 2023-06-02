#pragma once

#include "compute_radiance.h"

Image3 hw_2_1(const std::vector<std::string> &params);
Image3 hw_2_2(const std::vector<std::string> &params);
Image3 hw_2_3(const std::vector<std::string> &params);
Image3 hw_2_4(const std::vector<std::string> &params);
Image3 hw_2_5(const std::vector<std::string> &params);

// const unsigned int MAX_DEPTH = 10;    // defined in hw1.h
// const Real EPSILON = 1e-7;

// Functions below are specific to hw2 and will not be used later
namespace hw2 {


bool RayIntersectsTriangle(ray localRay, 
                           Triangle tri,
                           Real& dist,
                           Vector3& outIntersectionPoint,
                           Vector3& baryC);

bool RayIntersectsAny_Naive(ray localRay, 
                           std::vector<Triangle> tris,
                           Real& dist,  // should be inf when passed in
                           Vector3& outIntersectionPoint,
                           Vector3& baryC);

bool isVisible(Vector3& shadingPt, Vector3& lightPos, Scene& scene);

/**
 * @brief Compute the diffuse color at a given point in the Scene
 * 
 * @note For diffuse color, we don't need Shape information at all.
 * 
 * @param scene 
 * @param normal: normal of the point of interest
 * @param shadingPt: position of the point of interest 
 * @param diffuseMat: A pointer to Diffuse strcut 
 * @return Vector3 
 */
Vector3 computeDiffuseColor(Scene& scene, Vector3 normal, 
                    Vector3 shadingPt, Diffuse* diffuseMat);


// overarching callers
Vector3 computePixelColor(Scene& scene, ray& localRay, unsigned int recDepth=MAX_DEPTH);

// Used ONLY in hw_2_4
// hit box => (1,1,1), no hit => (0.5, 0.5, 0.5)
Vector3 bbox_color(std::vector<AABB> bboxes, ray& localRay);

}