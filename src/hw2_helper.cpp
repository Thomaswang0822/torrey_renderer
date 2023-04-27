#include "hw2.h"


bool isVisible(Vector3& shadingPt, Vector3& lightPos, Scene& scene) {
    double d = distance(shadingPt, lightPos);
    // shot ray from light to shadingPt
    ray lightRay(lightPos, shadingPt, true);
    // test hitting point
    // Baseline version: traverse all shapes and test
    Real hitDist = infinity<Real>();  // use to determine the closest hit
    Shape* hitObj = nullptr;
    checkRaySceneHit(lightRay, scene, hitDist, hitObj);
    return !bool(hitDist > EPSILON && hitDist < (1-EPSILON) * d);
}


Vector3 computeDiffuseColor(Scene& scene, Vector3 normal, 
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
            if (isVisible(shadingPt, ptLight->position, scene)) {
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


void checkRaySphereHit(ray localRay,
                       Sphere* sph,
                       Real& hitDist,
                       Shape*& hitObj) 
{
    Vector3 oc = localRay.origin() - sph->position;
    // a,b,c refer to those in at^2 + bt + c = 0
    auto a = length_squared(localRay.direction());
    auto half_b = dot(oc, localRay.direction());
    auto c = length_squared(oc) - sph->radius * sph->radius;
    auto discriminant = half_b*half_b - a*c;
    if (discriminant < 0) {
        // no hit
        return;
    } else {
        // minus because we want the closer hitting point -> smaller t
        double smallerRoot = (-half_b - sqrt(discriminant) ) / a;
        double biggerRoot = (-half_b + sqrt(discriminant) ) / a;
        if (smallerRoot < EPSILON) {
            // check the larger root: useful when Ray can travel inside Object
            if (biggerRoot < EPSILON) {return;}
            
            if (biggerRoot < hitDist) {
                // update with bigger root
                hitDist = biggerRoot;
                // crazy type-cast to make them fit; credit to ChatGPT
                hitObj = static_cast<Shape*>(static_cast<void*>(sph));
            }
        } else{
            if (smallerRoot < hitDist) {
                // update with bigger root
                hitDist = smallerRoot;
                // crazy type-cast to make them fit; credit to ChatGPT
                hitObj = static_cast<Shape*>(static_cast<void*>(sph));
            }
        }
        
        // std::cout << "Sphere hit: " << bool(hitObj == nullptr) << '\t' <<
        //     bool(sph == nullptr) << std::endl;
    }
}


void checkRayTriHit(ray localRay,
                    Triangle* tri,
                    Real& hitDist,
                    Shape*& hitObj)
{
    Vector3 h, s, q;
    Real a, f, u, v;
    h = cross(localRay.direction(), tri->e2);
    a = dot(tri->e1, h);

    if (a > -EPSILON && a < EPSILON) {
        return;    // This ray is parallel to this triangle.
    }
    f = 1.0 / a;
    s = localRay.origin() - tri->p0;
    u = f * dot(s, h);

    if (u < 0.0 || u > 1.0) {
        return;   // hit point not in triangle
    }

    q = cross(s, tri->e1);
    v = f * dot(localRay.direction(), q);

    if (v < 0.0 || u + v > 1.0) {
        return;   // hit point not in triangle
    }

    // At this stage we can compute t to find out where the intersection point is on the line.
    Real t = f * dot(tri->e2, q);

    if (t > EPSILON) // ray intersection
    {
        if (t >= hitDist) {return;}  // no a closer hit
        // only update storage variables before returning true
        hitDist = t;
        hitObj = static_cast<Shape*>(static_cast<void*>(tri));
        // std::cout << "Triangle hit" << bool(hitObj == nullptr) << '\t' <<
        //     bool(tri == nullptr) << std::endl;
    }
    else {// This means a line intersection but not a ray intersection.
        return;
    }
}


void checkRayShapeHit(ray localRay,
                    Shape& curr_shape,
                    Real& hitDist,
                    Shape*& hitObj)
{
    if (Sphere *sph = std::get_if<Sphere>(&curr_shape)) {
        // check ray-sphere intersection; auto update hitDist and hitObj
        checkRaySphereHit(localRay, sph, hitDist, hitObj);
    } else if (Triangle *tri = std::get_if<Triangle>(&curr_shape)) {
        // check ray triangle intersection
        checkRayTriHit(localRay, tri, hitDist, hitObj);
    } else {
        assert(false);
    }
}


void checkRaySceneHit(ray localRay,
                      Scene& scene,
                      Real& hitDist,
                      Shape*& hitObj)
{
    // Naive way: loop thru all shapes
    for (int i = 0; i < (int)scene.shapes.size(); i++) {
        Shape &curr_shape = scene.shapes[i];
        checkRayShapeHit(localRay, curr_shape, hitDist, hitObj);
    }
}


Vector3 computePixelColor(Scene& scene, ray& localRay, unsigned int recDepth)
{
    // Step 1: detect hit
    Real hitDist = infinity<Real>();  // use to determine the closest hit
    Shape* hitObj = nullptr;
    checkRaySceneHit(localRay, scene, hitDist, hitObj);
    if (hitDist > 1e9) {  // no hit
        return scene.background_color;
    }
    // std::cout << hitDist << '\t' << bool(hitObj == nullptr) << std::endl;
    assert(hitObj && "Bug: hitObj is a nullptr even when a hit is detected.");

    // Step 2: found hit -> get Material of hitObj
    //   Also compute normal depending on shape, in case mirror_ray()
    Material currMaterial;
    Vector3 hitNormal;
    Vector3 hitPt = localRay.at(hitDist);
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
        return computeDiffuseColor(scene, hitNormal, localRay.at(hitDist), diffuseMat);
    }
    else if (Mirror* mirrorMat = std::get_if<Mirror>(&currMaterial)) {
        // mirror refect ray and do recursion
        ray rayOut = mirror_ray(localRay, hitNormal, hitPt);
        Vector3* mirrorColor = std::get_if<Vector3>(&mirrorMat->reflectance);
        assert(mirrorColor && "Mirror material has reflectance not Vec3 RGB");
        return *mirrorColor // color at current hitting pt
            * computePixelColor(scene, rayOut, recDepth=recDepth-1);   // element-wise mutiply
    }
    else {
        std::cout << "Material not Diffuse or Mirror; will implement later" 
            << std::endl;
    }


    return Vector3(0.0, 0.0, 0.0);
}



bool RayIntersectsTriangle(ray localRay, 
                           Triangle tri,
                           Real& dist,
                           Vector3& outIntersectionPoint,
                           Vector3& baryC)
{
    Vector3 h, s, q;
    Real a, f, u, v;
    h = cross(localRay.direction(), tri.e2);
    a = dot(tri.e1, h);

    if (a > -EPSILON && a < EPSILON) {
        return false;    // This ray is parallel to this triangle.
    }
    f = 1.0 / a;
    s = localRay.origin() - tri.p0;
    u = f * dot(s, h);

    if (u < 0.0 || u > 1.0) {
        return false;   // hit point not in triangle
    }

    q = cross(s, tri.e1);
    v = f * dot(localRay.direction(), q);

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


bool RayIntersectsAny_Naive(ray localRay, 
                           std::vector<Triangle> tris,
                           Real& dist,  // should be inf when passed in
                           Vector3& outIntersectionPoint,
                           Vector3& baryC)
{
    Real currT;
    Vector3 currHitPos, currBaryC;
    // Brute-force: look at each triangle and update only if (hit && closer)
    for (Triangle tri : tris) {
        if (RayIntersectsTriangle(localRay, tri, currT, currHitPos, currBaryC)) {
            if (currT < dist) {
                dist = currT;
                outIntersectionPoint = Vector3(currHitPos);
                baryC = Vector3(currBaryC);
            }
        }
    }
    return bool(dist < infinity<Real>());
}


bool BVH_isVisible(Vector3& shadingPt, Vector3& lightPos, Scene& scene, BVH_node root) {
    double d = distance(shadingPt, lightPos);
    // shot ray from light to shadingPt
    ray lightRay(lightPos, shadingPt, true);
    // test hitting point
    // Baseline version: traverse all shapes and test
    Real hitDist = infinity<Real>();  // use to determine the closest hit
    Shape* hitObj = nullptr;
    // hit => not visible (shadow)
    return !root.hit(lightRay, EPSILON, (1-EPSILON) * d, scene, hitDist, hitObj);
}

// the only change is calling BVH_isVisible()
Vector3 BVH_DiffuseColor(Scene& scene, Vector3 normal, 
                    Vector3 shadingPt, Diffuse* diffuseMat, BVH_node root)
{
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
            if (BVH_isVisible(shadingPt, ptLight->position, scene, root)) {
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


Vector3 BVH_PixelColor(Scene& scene, ray& localRay, BVH_node root, unsigned int recDepth) {
    // Step 1 BVH UPDATE: detect hit. 
    Real hitDist = infinity<Real>();  // use to determine the closest hit
    Shape* hitObj = nullptr;
    // checkRaySceneHit(localRay, scene, hitDist, hitObj);
    root.hit(localRay, EPSILON, infinity<Real>(), scene, hitDist, hitObj);
    if (hitDist > 1e9) {  // no hit
        return scene.background_color;
    }
    // std::cout << hitDist << '\t' << bool(hitObj == nullptr) << std::endl;
    assert(hitObj && "Bug: hitObj is a nullptr even when a hit is detected.");

    // Step 2: found hit -> get Material of hitObj
    //   Also compute normal depending on shape, in case mirror_ray()
    Material currMaterial;
    Vector3 hitNormal;
    Vector3 hitPt = localRay.at(hitDist);
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

        /* // REMOVE: 
        if (hitTri->material_id == 1) {
            // mirror
            cout << "Hit the mirror triangle" << endl;
            cout << "position: " << hitTri->p0 << hitTri->p1 << hitTri->p2 << endl;
            cout << "normal: " << hitTri->n0 << hitTri->n1 << hitTri->n2 << endl;
            cout << "Triangle normal: " << hitTri->normal << endl;
        } */
        
    } else {
        assert(false && "hitObj is neither Sphere or Triangle. Should NEVER happen.");
    }

    // Step 3 BVH UPDATE: act according to Material (instead of Shape)
    if (Diffuse* diffuseMat = std::get_if<Diffuse>(&currMaterial)) {
        // no recursion, compute diffuse color
        return BVH_DiffuseColor(scene, hitNormal, localRay.at(hitDist), diffuseMat, root);
    }
    else if (Mirror* mirrorMat = std::get_if<Mirror>(&currMaterial)) {
        // mirror refect ray and do recursion
        ray rayOut = mirror_ray(localRay, hitNormal, hitPt);
        Vector3* mirrorColor = std::get_if<Vector3>(&mirrorMat->reflectance);
        assert(mirrorColor && "Mirror material has reflectance not Vec3 RGB");
        return *mirrorColor // color at current hitting pt
            * BVH_PixelColor(scene, rayOut, root, recDepth=recDepth-1);   // element-wise mutiply
    }
    else {
        std::cout << "Material not Diffuse or Mirror; will implement later" 
            << std::endl;
    }


    return Vector3(0.0, 0.0, 0.0);
}


