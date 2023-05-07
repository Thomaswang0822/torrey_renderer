#include "helper.h"

using namespace std;

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
        // hw 3_2 UPDATE: interpolate vertex normal
        Vector3 shadingNormal = tri->shading_normal(u,v);
        rec.set_face_normal(localRay, shadingNormal);
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
    if (Sphere *sph = get_if<Sphere>(&curr_shape)) {
        // check ray-sphere intersection; auto update rec and hitObj
        checkRaySphereHit(localRay, sph, rec, hitObj);
    } else if (Triangle *tri = get_if<Triangle>(&curr_shape)) {
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


Vector3 sample_point(const Shape* lightObj, pcg32_state& rng) {
    Real u1 = next_pcg32_real<Real>(rng);
    Real u2 = next_pcg32_real<Real>(rng);
    if (const Sphere *sph = get_if<Sphere>(lightObj)) {
        // sample from sphere: 
        // elevation angle theta; azumith angle phi
        double theta = acos(1.0 - 2 * u1);
        double phi = c_TWOPI * u2;
        // convert sphercial coor to xyz position
        // c + r (sin θ cos φ,sin θ sin φ, cos θ).
        return Vector3(
            sin(theta) * cos(phi),
            sin(theta) * sin(phi),
            cos(theta)
        ) * sph->radius + sph->position;

    } else if (const Triangle *tri = get_if<Triangle>(lightObj)) {
        Real b1 = 1 - sqrt(u1);
        Real b2 = u2 * sqrt(u1);
        // use baryC to get position
        return Vector3(
            (1-b1-b2) * tri->p0 +
            b1 * tri->p1 + 
            b2 * tri->p2
        );
    } else {
        assert(false);
    }
}


Vector3 areaLight_contribution(const Shape* lightObj, Hit_Record& rec, 
            const Vector3& lightPos, const Vector3& Kd, 
            const Vector3& I, const Vector3& nx)
{

    // These 2 are unique for every sample point.
    
    // normalized direction: shadingPt to light position
    Vector3 l = normalize(lightPos - rec.pos);
    // distance squared: shadingPt to light position
    Real dsq = distance_squared(rec.pos, lightPos);

    // nx also, but it depend on Sphere/Triangle, so we handle it outside.

    return (Kd * I * c_INVPI / dsq) // const
        * std::max(dot(rec.normal, l), 0.0) // max (ns · l, 0)
        * std::max(dot(-nx, l), 0.0);  // max (−nx · l, 0)
}
