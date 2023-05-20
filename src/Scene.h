#pragma once

#include "all_utils.h"
// Put all geometric & physical data in the scene here.
#include "ray.h"
#include "Hit_Record.h"
#include "parse_scene.h"
#include "material.h"

using namespace std;

struct Camera
{
    Vector3 origin;
    Vector3 lookat;
    Vector3 up;
    Real vfov;
    Vector3 horizontal, vertical;
    Vector3 lower_left_corner;

    Camera(
        Vector3 lookfrom,
        Vector3 lookat,
        Vector3 vup,
        double vfov, // vertical field-of-view in degrees
        int width,
        int height  // w and h of image
    ) : origin(lookfrom), lookat(lookat), up(vup), vfov(vfov)
                                            // origin(lookfrom)
    {
        Real aspect_ratio = Real(width) / height;
        auto theta = radians(vfov);
        auto h = tan(theta / 2);
        auto viewport_height = 2.0 * h;
        auto viewport_width = aspect_ratio * viewport_height;

        auto w = normalize(lookfrom - lookat); // opposite camera focus direction
        auto u = normalize(cross(vup, w));     // "right"
        auto v = cross(w, u);                  // projected "up"

        horizontal = viewport_width * u;
        vertical = viewport_height * v;
        lower_left_corner = lookfrom - horizontal / 2.0 - vertical / 2.0 - w;
    }

    Camera(const ParsedCamera& pCam) :
        // just call default constructor
        Camera(pCam.lookfrom, pCam.lookat, pCam.up, pCam.vfov, pCam.width, pCam.height)
    {};

    ray get_ray(double offset_u, double offset_v) const
    {
        return ray(
            origin,
            lower_left_corner + offset_u * horizontal + offset_v * vertical - origin);
    }
};

// Geometric property
/**
 * @brief Axis-aligned bounding box. Copied from RTNW, Listing 8
 * 
 */
struct AABB {
    Vector3 minimum;
    Vector3 maximum;

    AABB() {};
    AABB(const Vector3& a, const Vector3& b) : 
        minimum(a), maximum(b) {};

    bool hit(const ray& r, double t_min, double t_max) const {
        // for each xyz axis
        for (int a = 0; a < 3; a++) {
            auto invD = 1.0f / r.dir[a];
            auto t0 = (minimum[a] - r.orig[a]) * invD;
            auto t1 = (maximum[a] - r.orig[a]) * invD;
            if (invD < 0.0f)
                std::swap(t0, t1);
            t_min = t0 > t_min ? t0 : t_min;
            t_max = t1 < t_max ? t1 : t_max;
            if (t_max < t_min)
                return false;
        }
        return true;
    }

    Real surfaceA() {
        Vector3 diff = maximum - minimum;
        return 2 * (diff.x * diff.y + diff.x * diff.z + diff.y * diff.z);
    }

    AABB merge(AABB& neighbor) {
        Vector3 newMin = min(minimum, neighbor.minimum);
        Vector3 newMax = max(maximum, neighbor.maximum);

        return AABB(newMin, newMax);
    }

    Vector3 center() {
        return 0.5 * (minimum + maximum);
    }
};

struct ShapeBase {
    int material_id = -1;
    int area_light_id = -1;
};

struct Sphere : public ShapeBase {
    Vector3 position;
    Real radius;
    AABB box;   // has default constructor

    Sphere(int mat_id, int light_id, Vector3 pos, Real r) :
        ShapeBase{mat_id, light_id},
        position(pos), radius(r)
    {
        // create bbox during instantiation
        // Vector3 - Real has been overloaded
        box = AABB(pos-r, pos+r);
    };

    void get_sphere_uv(Vector3 normal, double& u, double& v) {
        auto theta = acos(-normal.y);
        auto phi = atan2(-normal.z, normal.x) + c_PI;

        u = phi / (2 * c_PI);
        v = theta / c_PI;
    }

    Vector3 normal_at(Vector3 location) const {
        return normalize(location - position);
    }

};

struct TriangleMesh : public ShapeBase {
    std::vector<Vector3> positions;
    std::vector<Vector3i> indices;
    std::vector<Vector3> normals;
    std::vector<Vector2> uvs;
    int size;
    Real totalArea;
    std::vector<Real> areaCDF;

    // Give it a constructor in order to sample area light
    TriangleMesh() {};
    TriangleMesh(const ParsedTriangleMesh& pMesh) :
        ShapeBase{pMesh.material_id, pMesh.area_light_id},
        positions(pMesh.positions), indices(pMesh.indices), 
        normals(pMesh.normals), uvs(pMesh.uvs),
        size((int)pMesh.indices.size()),
        areaCDF(pMesh.indices.size() + 1, 0.0)
    {
        // areaCDF will be populated in Scene constructor after
        // all Triangles are created
    }

    // given a Unif(0,1), return it's corresponding sample triangle
    int which_tri(double rd) {
        // upper_bound because we want
        // [0, 0.4, 0.8, 1.0] searching 0.4 should gives 1
        auto it = std::upper_bound(areaCDF.begin(), areaCDF.end(), rd);
        std::size_t index = std::distance(areaCDF.begin(), it - 1);
        return static_cast<int>(index);
    }
};

struct Triangle : public ShapeBase{
    Vector3 p0, p1, p2;  // vertices position
    Vector3 n0, n1, n2;
    Vector3 normal;  // may need triangle normal
    Vector3 e1, e2;  // p1-p0, p2-p0; not normalized
    double area;
    int face_id = -1;
    int mesh_id = -1;   // in order to retrieve material and light id
    AABB box;   // has default constructor
    Vector2 uv0, uv1, uv2;  // uv coordinates; possibly null
    bool hasUV = false;

    // naive constructor; used in hw_2_1 and hw_2_2
    Triangle(Vector3 pos0, Vector3 pos1, Vector3 pos2) :
        p0(pos0), p1(pos1), p2(pos2),
        e1(pos1 - pos0), e2(pos2 - pos0) {}

    // ordinary constructor; used whem creating a Scene
    Triangle(int face_index, TriangleMesh* mesh, int mesh_index) :
        face_id(face_index), mesh_id(mesh_index)
    {
        // get indices
        Vector3i id3 = mesh->indices[face_index];
        // "repeat" naive constructor
        p0 = mesh->positions[id3[0]];
        p1 = mesh->positions[id3[1]];
        p2 = mesh->positions[id3[2]];
        e1 = p1 - p0;  e2 = p2 - p0;
        area = 0.5 * length(cross(e1, e2));
        // write normals
        normal = normalize(cross(e1, e2));
        if (mesh->normals.size() > 0) {
            n0 = mesh->normals[id3[0]];
            n1 = mesh->normals[id3[1]];
            n2 = mesh->normals[id3[2]];
        } else {
            // just use Triangle normal
            n0 = normal; n1 = normal; n2 = normal;
        }
        // material id
        material_id = mesh->material_id;
        // create bbox during instantiation
        Vector3 minPt = min(p0, min(p1, p2));
        Vector3 maxPt = max(p0, max(p1, p2));
        box = AABB(minPt, maxPt);
        // deal with uvs
        if (mesh->uvs.size() > 0) {
            uv0 = mesh->uvs[id3[0]];
            uv1 = mesh->uvs[id3[1]];
            uv2 = mesh->uvs[id3[2]];
            hasUV = true;
        }
        // set material_id and light_id
        material_id = mesh->material_id;
        area_light_id = mesh->area_light_id;
    }

    // uv of a point requires vertex uv info => non-static
    void get_tri_uv(Real b1, Real b2, double& rec_u, double& rec_v) {
        assert(hasUV && "Calling get_tri_uv() on Triangle without uv");

        rec_u = (1.0 - b1 - b2) * uv0.x + b1 * uv1.x + b2 * uv2.x;
        rec_v = (1.0 - b1 - b2) * uv0.y + b1 * uv1.y + b2 * uv2.y;
    }

    // interpolate 3 vertex normals with baryC
    inline Vector3 shading_normal(double b1, double b2) {
        return normalize(
            (1.0-b1-b2) * n0 +
            b1 * n1 +
            b2 * n2
        );
    }
};

using Shape = std::variant<Sphere, Triangle>;

// Material property defined in material.h

// Light property
struct PointLight {
    Vector3 position;
    Vector3 intensity;
};

struct DiffuseAreaLight {
    int shape_id;  // points to a Sphere or the first Triangle in a mesh
    Vector3 radiance;
};

using Light = std::variant<PointLight, DiffuseAreaLight>;

// helper functions
inline int get_material_id(const Shape &shape) {
    return std::visit([&](const auto &s) { return s.material_id; }, shape);
}
inline int get_area_light_id(const Shape &shape) {
    return std::visit([&](const auto &s) { return s.area_light_id; }, shape);
}
inline bool is_light(const Shape &shape) {
    return get_area_light_id(shape) >= 0;
}


// A scene encapsulating everything above
struct Scene {
    Scene(const ParsedScene &scene);

    Camera camera;
    int width, height;
    std::vector<Shape> shapes;
    std::vector<Material> materials;
    std::vector<Light> lights;
    Vector3 background_color;

    int samples_per_pixel;
    // For the Triangle in the shapes to reference to.
    std::vector<TriangleMesh> meshes;
};


// Create an AABB of a primative; used in hw_2_4
AABB bounding_box(Shape curr_shape);

// Starting from hw_2_5, an AABB will be attached to a Shape during creation. 
// Returns a reference to the box.
AABB& get_bbox(std::shared_ptr<Shape> curr_shape);

// Create an AABB that encloses the given 2 smaller AABBs
AABB surrounding_box(AABB box0, AABB box1);

