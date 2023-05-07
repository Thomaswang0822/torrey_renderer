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

/**
 * @brief Given a shape, uniformly sample a point on the surface
 * 
 * @param lightObj pointer to the area-lighted Shape, can be either Sphere or Triangle
 * @param rng 
 * @return Vector3 
 */
Vector3 sample_point(const Shape* lightObj, pcg32_state& rng);

/**
 * @brief Given a area-lighted primitive object, return the UNSCALED RGB light
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
