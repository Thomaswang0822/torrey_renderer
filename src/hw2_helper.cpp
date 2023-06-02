#include "hw2.h"

using namespace hw2;

bool hw2::isVisible(Vector3& shadingPt, Vector3& lightPos, Scene& scene) {
    double d = distance(shadingPt, lightPos);
    // shot ray from light to shadingPt
    ray lightRay(lightPos, shadingPt, true);
    // test hitting point
    // Baseline version: traverse all shapes and test
    Hit_Record rec;  // use to determine the closest hit
    Shape* hitObj = nullptr;
    checkRaySceneHit(lightRay, scene, rec, hitObj);
    return !bool(rec.dist > EPSILON && rec.dist < (1-EPSILON) * d);
}


Vector3 hw2::computeDiffuseColor(Scene& scene, Vector3 normal, 
                    Vector3 shadingPt, Diffuse* diffuseMat) {
    Vector3 result = Vector3(0.0, 0.0, 0.0);

    // Get Kd: the reflectance of Diffuse
    Vector3* diffuseColor = std::get_if<Vector3>(&diffuseMat->reflectance);
    assert(diffuseColor && "Diffuse material has reflectance not Vec3 RGB");
    Vector3 Kd = *diffuseColor;

    // attributes that are different for each light
    Vector3 l;  // normalized shadingPt to light position
    Real dsq;  // distance squared: shadingPt to light position
    for (Light light : scene.lights) {
        // check point light vs area light
        if (PointLight* ptLight = std::get_if<PointLight>(&light)) {
            l = normalize(ptLight->position - shadingPt);
            dsq = distance_squared(shadingPt, ptLight->position);
            if (hw2::isVisible(shadingPt, ptLight->position, scene)) {
                // abs(dot(normal, l)) can be replaced by using helper function below
                // (incomingRayOutside(-l, normal))? dot(normal, -l):dot(normal, -l);
                // but obviously it's unnecessary here.
                result += Kd * std::max( abs(dot(normal, l)), 0.0 ) * 
                    c_INVPI * ptLight->intensity / dsq;
            }
        } 
        else if (DiffuseAreaLight* areaLight = std::get_if<DiffuseAreaLight>(&light)) {
            std::cout << "Area light not implemented yet; will implement later" 
                << std::endl;
            UNUSED(areaLight);
        }
        
    }
    return result;
}


Vector3 hw2::computePixelColor(Scene& scene, ray& localRay, unsigned int recDepth)
{
    // Step 1: detect hit
    Hit_Record rec;  // use to determine the closest hit
    Shape* hitObj = nullptr;
    checkRaySceneHit(localRay, scene, rec, hitObj);
    if (rec.dist > 1e9) {  // no hit
        return scene.background_color;
    }
    assert(hitObj && "Bug: hitObj is a nullptr even when a hit is detected.");

    // Step 2: found hit -> get Material of hitObj
    //   Also compute normal depending on shape, in case mirror_ray()
    Material currMaterial;
    Vector3 hitNormal;
    Vector3 hitPt = rec.pos;
    if (Sphere* hitSph = std::get_if<Sphere>(hitObj)) {
        // get material
        currMaterial = scene.materials[hitSph->material_id];
        // get normal of a sphere
        hitNormal = normalize(hitPt - hitSph->position);
    } else if (Triangle* hitTri = std::get_if<Triangle>(hitObj)) {
        // get material
        currMaterial = scene.materials[hitTri->material_id];
        // get normal of a triangle
        hitNormal = hitTri->normal;
        // std::cout << "Hit a Triangle" << std::endl;
        
    } else {
        assert(false && "hitObj is neither Sphere or Triangle. Should NEVER happen.");
    }

    // Step 3: act according to Material (instead of Shape)
    if (Diffuse* diffuseMat = std::get_if<Diffuse>(&currMaterial)) {
        // no recursion, compute diffuse color
        return hw2::computeDiffuseColor(scene, hitNormal, hitPt, diffuseMat);
    }
    else if (Mirror* mirrorMat = std::get_if<Mirror>(&currMaterial)) {
        // mirror refect ray and do recursion
        ray rayOut = mirror_ray(localRay, hitNormal, hitPt);
        Vector3* mirrorColor = std::get_if<Vector3>(&mirrorMat->reflectance);
        assert(mirrorColor && "Mirror material has reflectance not Vec3 RGB");
        return *mirrorColor // color at current hitting pt
            * hw2::computePixelColor(scene, rayOut, recDepth=recDepth-1);   // element-wise mutiply
    }
    else {
        std::cout << "Material not Diffuse or Mirror; will implement later" 
            << std::endl;
    }


    return Vector3(0.0, 0.0, 0.0);
}



bool hw2::RayIntersectsTriangle(ray localRay, 
                           Triangle tri,
                           Real& dist,
                           Vector3& outIntersectionPoint,
                           Vector3& baryC)
{
    Vector3 h, s, q;
    Real a, f, u, v;
    h = cross(localRay.dir, tri.e2);
    a = dot(tri.e1, h);

    if (a > -EPSILON && a < EPSILON) {
        return false;    // This ray is parallel to this triangle.
    }
    f = 1.0 / a;
    s = localRay.orig - tri.p0;
    u = f * dot(s, h);

    if (u < 0.0 || u > 1.0) {
        return false;   // hit point not in triangle
    }

    q = cross(s, tri.e1);
    v = f * dot(localRay.dir, q);

    if (v < 0.0 || u + v > 1.0) {
        return false;   // hit point not in triangle
    }

    // At this stage we can compute t to find out where the intersection point is on the line.
    Real t = f * dot(tri.e2, q);

    if (t > EPSILON) // ray intersection
    {
        // only update storage variables before returning true
        dist = t;
        outIntersectionPoint = localRay.at(t);
        baryC = {1.0-u-v, u, v};
        return true;
    }
    else {// This means a line intersection but not a ray intersection.
        return false;
    }
}


bool hw2::RayIntersectsAny_Naive(ray localRay, 
                           std::vector<Triangle> tris,
                           Real& dist,  // should be inf when passed in
                           Vector3& outIntersectionPoint,
                           Vector3& baryC)
{
    Real currT;
    Vector3 currHitPos, currBaryC;
    // Brute-force: look at each triangle and update only if (hit && closer)
    for (Triangle tri : tris) {
        if (hw2::RayIntersectsTriangle(localRay, tri, currT, currHitPos, currBaryC)) {
            if (currT < dist) {
                dist = currT;
                outIntersectionPoint = Vector3(currHitPos);
                baryC = Vector3(currBaryC);
            }
        }
    }
    return bool(dist < infinity<Real>());
}


Vector3 hw2::bbox_color(vector<AABB> bboxes, ray& localRay) {
    for (AABB bbox : bboxes) {
        if (bbox.hit(localRay, 0.0, infinity<double>())) {
            return Vector3(1.0, 1.0, 1.0);
        }
    }
    return Vector3(0.5, 0.5, 0.5);
}