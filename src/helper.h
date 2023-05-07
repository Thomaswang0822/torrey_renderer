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
 * @param hitObj pointer to the Shape, can be either Sphere or Triangle
 * @param rng 
 * @return Vector3 
 */
Vector3 sample_point(const Shape* hitObj, pcg32_state& rng);
