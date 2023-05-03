#include "helper.h"


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
