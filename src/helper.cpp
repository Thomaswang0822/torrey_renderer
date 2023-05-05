#include "helper.h"


void checkRaySphereHit(ray localRay,
                       Sphere* sph,
                       Hit_Record& rec,
                       Shape*& hitObj) 
{
    Vector3 oc = localRay.origin() - sph->position;
    // a,b,c refer to those in at^2 + bt + c = 0
    auto a = length_squared(localRay.direction());
    auto half_b = dot(oc, localRay.direction());
    auto c = length_squared(oc) - sph->radius * sph->radius;
    auto discriminant = half_b*half_b - a*c;
    Real root;
    if (discriminant < 0) {
        // no hit
        return;
    } else {
        // minus because we want the closer hitting point -> smaller t
        double smallerRoot = (-half_b - sqrt(discriminant) ) / a;
        double biggerRoot = (-half_b + sqrt(discriminant) ) / a;

        // 1st hit too close, check 2nd hit: useful when Ray can travel inside Object
        if (smallerRoot < EPSILON) {
            if (biggerRoot < EPSILON) {return;}  // 2nd also too close
            
            if (biggerRoot < rec.dist) {  // 2nd hit is a valid update
                // update with bigger root
                root = biggerRoot;
            } else {  return; }
        } else{
            if (smallerRoot < rec.dist) { // 1st hit is a valid update
                // update with bigger root
                root = smallerRoot;    
            } else {  return; }
        }
    }

    // Valid Update (found a closer hit) when reaching here
    rec.dist = root;
    rec.pos = localRay.at(root);
    Vector3 outward_normal = (rec.pos - sph->position) / sph->radius;
    rec.set_face_normal(localRay, outward_normal);
    sph->get_sphere_uv(outward_normal, rec.u, rec.v);
    rec.mat_id = sph->material_id;
    // crazy type-cast to make them fit; credit to ChatGPT
    hitObj = static_cast<Shape*>(static_cast<void*>(sph));
}


void checkRayTriHit(ray localRay,
                    Triangle* tri,
                    Hit_Record& rec,
                    Shape*& hitObj)
{
    // local u,v are baryC of a Triangle, i.e. (1-u-v, u, v)
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
        if (t >= rec.dist) {return;}  // no a closer hit
        // only update storage variables before returning true
        rec.dist = t;
        rec.pos = localRay.at(t);
        rec.set_face_normal(localRay, tri->normal);
        // if the mesh contains UV coordinates
        if (tri->hasUV) {
            tri->get_tri_uv(u, v, rec.u, rec.v);
        } else { // does not come with UV; use baryC
            rec.u = u; rec.v = v;
        }
        rec.mat_id = tri->material_id;
        // crazy type-cast to make them fit; credit to ChatGPT
        hitObj = static_cast<Shape*>(static_cast<void*>(tri));
    }
    else {// This means a line intersection but not a ray intersection.
        return;
    }
}


void checkRayShapeHit(ray localRay,
                    Shape& curr_shape,
                    Hit_Record& rec,
                    Shape*& hitObj)
{
    if (Sphere *sph = std::get_if<Sphere>(&curr_shape)) {
        // check ray-sphere intersection; auto update rec and hitObj
        checkRaySphereHit(localRay, sph, rec, hitObj);
    } else if (Triangle *tri = std::get_if<Triangle>(&curr_shape)) {
        // check ray triangle intersection
        checkRayTriHit(localRay, tri, rec, hitObj);
    } else {
        assert(false);
    }
}


void checkRaySceneHit(ray localRay,
                      Scene& scene,
                      Hit_Record& rec,
                      Shape*& hitObj)
{
    // Naive way: loop thru all shapes
    for (int i = 0; i < (int)scene.shapes.size(); i++) {
        Shape &curr_shape = scene.shapes[i];
        checkRayShapeHit(localRay, curr_shape, rec, hitObj);
    }
}
