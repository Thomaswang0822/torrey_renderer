#include "helper.h"

using namespace std;

void checkRaySphereHit(ray localRay,
                       Sphere* sph,
                       Hit_Record& rec,
                       Shape*& hitObj) 
{
    Vector3 oc = localRay.orig - sph->position;
    // a,b,c refer to those in at^2 + bt + c = 0
    auto a = length_squared(localRay.dir);
    auto half_b = dot(oc, localRay.dir);
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
    h = cross(localRay.dir, tri->e2);
    a = dot(tri->e1, h);

    if (a > -EPSILON && a < EPSILON) {
        return;    // This ray is parallel to this triangle.
    }
    f = 1.0 / a;
    s = localRay.orig - tri->p0;
    u = f * dot(s, h);

    if (u < 0.0 || u > 1.0) {
        return;   // hit point not in triangle
    }

    q = cross(s, tri->e1);
    v = f * dot(localRay.dir, q);

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


Vector3 Triangle_sample(const Triangle* tri, pcg32_state& rng, int which_part) {
    Real u1 = next_pcg32_real<Real>(rng);
    Real u2 = next_pcg32_real<Real>(rng);
    Real b1 = 1 - sqrt(u1);
    Real b2 = u2 * sqrt(u1);
    if (which_part == -1) {    
        // use baryC to get position
        return Vector3(
            (1-b1-b2) * tri->p0 +
            b1 * tri->p1 + 
            b2 * tri->p2
        );
    } else {
        Vector3 p0, p1, p2;  // vertex position of sub-Triangle we pick
        switch (which_part)
        {
        case 0:
            p0 = tri->p0;
            p1 = 0.5 * (p0 + tri->p1);
            p2 = 0.5 * (p0 + tri->p2);
            break;
        case 1:
            p1 = tri->p1;
            p0 = 0.5 * (p1 + tri->p0);
            p2 = 0.5 * (p1 + tri->p2);
            break;
        case 2:
            p2 = tri->p2;
            p0 = 0.5 * (p2 + tri->p0);
            p1 = 0.5 * (p2 + tri->p1);
            break;
        case 3:
            // all 3 are mid points, vertex order is flipped and but when don't
            // deal with normal, so no problem
            p0 = 0.5 * (tri->p1 + tri->p2);
            p1 = 0.5 * (tri->p0 + tri->p2);
            p2 = 0.5 * (tri->p0 + tri->p1);
            break;
        default:
            Error("Wrong Triangle stratification index. Should be 0 to 3")
            break;
        }
        // use baryC on this sub-Triangle;
        return Vector3(
            (1-b1-b2) * p0 +
            b1 * p1 + 
            b2 * p2
        );
    }
}


Vector3 SphTri_sample(Triangle* tri, pcg32_state& rng, Hit_Record& rec)
{
    // construct the Spherical Triangle
    SphTriangle sphtri(tri, rec.pos);

    // sample a local position on the unit sphere
    Vector3 p = sphtri.local_sample(rng);

    // send p back to the triangle
    ray ray_back(rec.pos, rec.pos + p, true);
    Hit_Record rec_back;
    Shape* hitObj = nullptr;
    checkRayTriHit(ray_back, tri, rec_back, hitObj);
    if (hitObj == nullptr) {
        // above function fails, meaning tri.normal is perpendicular to xp
        // just return tri angle center. visibility check will make contribution to 0
        return (tri->p0 + tri->p1 + tri->p2) / 3.0;
    }
    /* Triangle* tri1 = get_if<Triangle>(hitObj);
    if (tri1->face_id != tri->face_id)
        std::cout << tri1->face_id << tri->face_id << endl; */
    return Vector3(rec_back.pos);
    // return tri->p0 * (1.0 - rec.u - rec.v) +
    //     tri->p1 * rec.u + tri->p2 * rec.v;
}


Vector3 Sphere_sample(const Sphere* sph, pcg32_state& rng, int idx, int ct) {
    Real u1 = next_pcg32_real<Real>(rng);
    Real u2 = next_pcg32_real<Real>(rng);
    // elevation angle theta; azumith angle phi
    double theta = acos(1.0 - 2 * u1);
    // for stratified sample: offset to the particular "orange slice".
    double phi = c_TWOPI * (u2 + Real(idx)/ ct);

    // convert sphercial coor to xyz position
    // c + r (sin θ cos φ,sin θ sin φ, cos θ).
    return Vector3(
        sin(theta) * cos(phi),
        sin(theta) * sin(phi),
        cos(theta)
    ) * sph->radius + sph->position;
}


Vector3 Sphere_sample_cone(const Sphere* sph, pcg32_state& rng, 
                const Real& cos_theta_max, const Vector3& cp) {
    assert(cos_theta_max > 0.0 && cos_theta_max < 1.0 
        && "cos(theta max) should be between 0 and 1.");
    Real u1 = next_pcg32_real<Real>(rng);
    Real u2 = next_pcg32_real<Real>(rng);
    // elevation angle theta; azumith angle phi
    double theta = acos(1.0 - u1 * (1.0 - cos_theta_max));
    double phi = c_TWOPI * u2;

    // This is the point under the Sphere local space
    // i.e. center at 0, up vector is (0,1,0)
    Vector3 local_p = Vector3(
        sin(theta) * cos(phi),
        sin(theta) * sin(phi),
        cos(theta)
    ) * sph->radius;
    
    // create the orthonormal basis with cp as up vector
    Basis world_basis = Basis::orthonormal_basis(cp);
    // plug in formula; turn into world coordinate
    return sph->position + 
        local_p.x * world_basis.u +
        local_p.y * world_basis.v_up +
        local_p.z * world_basis.w;
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


Vector3 dir_cos_sample(pcg32_state& rng, Basis& basis) {
    double u1 = next_pcg32_real<double>(rng);
    double u2 = next_pcg32_real<double>(rng);

    Real z = sqrt(1.0 - u2);  // also cos(theta)
    Real phi = c_TWOPI * u1;
    Real x = cos(phi) * sqrt(u2);
    Real y = sin(phi) * sqrt(u2);

    // turn to world space
    Vector3 local_pos(x,y,z);
    return basis.local2world(local_pos);
}


Vector3 dir_Phong_sample(pcg32_state& rng, Basis& basis, Real alpha) {
    double u1 = next_pcg32_real<double>(rng);
    double u2 = next_pcg32_real<double>(rng);

    Real phi = c_TWOPI * u2;
    Real cos_theta = pow(1.0-u1, 1.0/(alpha+1.0));  // derived in quiz problem
    // assert(0.0 <= cos_theta && cos_theta <= 1.0 && "cos_theta is wrong");
    Real sin_theta = sqrt(1.0 - cos_theta*cos_theta);

    Real x = sin_theta * cos(phi);
    Real y = sin_theta * sin(phi);
    Real z = cos_theta;

    // turn to world space
    Vector3 local_pos(x,y,z);
    return basis.local2world(local_pos);
}


Vector3 dir_GGX_sample(pcg32_state& rng, Basis& basis, Real exponent) {
    double u1 = next_pcg32_real<double>(rng);
    double u2 = next_pcg32_real<double>(rng);

    Real phi = c_TWOPI * u2;

    Real alpha_sq = 2.0 / (exponent + 2.0);
    Real theta = tan(sqrt( alpha_sq * u1 / (1.0 - u1) ));
    Real sin_theta = sin(theta); Real cos_theta = cos(theta);

    Real x = sin_theta * cos(phi);
    Real y = sin_theta * sin(phi);
    Real z = cos_theta;

    // turn to world space
    Vector3 local_pos(x,y,z);
    return basis.local2world(local_pos);
}