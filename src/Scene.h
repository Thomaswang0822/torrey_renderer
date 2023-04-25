#pragma once

#include "torrey.h"
#include "vector.h"
#include "ray.h"
#include "parse_scene.h"
#include "material.h"

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
            auto invD = 1.0f / r.direction()[a];
            auto t0 = (minimum[a] - r.origin()[a]) * invD;
            auto t1 = (maximum[a] - r.origin()[a]) * invD;
            if (invD < 0.0f)
                std::swap(t0, t1);
            t_min = t0 > t_min ? t0 : t_min;
            t_max = t1 < t_max ? t1 : t_max;
            if (t_max < t_min)
                return false;
        }
        return true;
    }
};

struct ShapeBase {
    int material_id = -1;
    int area_light_id = -1;
    AABB box;   // has default constructor
};

struct Sphere : public ShapeBase {
    Vector3 position;
    Real radius;

    Sphere(int mat_id, int light_id, Vector3 pos, Real r) :
        ShapeBase{mat_id, light_id},
        position(pos), radius(r)
    {
        // create bbox during instantiation
        // Vector3 - Real has been overloaded
        box = AABB(pos-r, pos+r);
    };
};

struct TriangleMesh : public ShapeBase {
    std::vector<Vector3> positions;
    std::vector<Vector3i> indices;
    std::vector<Vector3> normals;
    std::vector<Vector2> uvs;
};

struct Triangle : public ShapeBase{
    Vector3 p0, p1, p2;  // vertices position
    Vector3 n0, n1, n2;
    Vector3 normal;  // may need triangle normal
    Vector3 e1, e2;  // p1-p0, p2-p0; not normalized
    int face_id = -1;
    int mesh_id = -1;   // in order to retrieve material and light id

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
        // write normals
        n0 = mesh->normals[id3[0]];
        n1 = mesh->normals[id3[0]];
        n2 = mesh->normals[id3[0]];
        normal = normalize(cross(e1, e2));
        // material id
        material_id = mesh->material_id;
        // create bbox during instantiation
        Vector3 minPt(
            std::min(std::min(p0.x, p1.x), p2.x),
            std::min(std::min(p0.y, p1.y), p2.y),
            std::min(std::min(p0.z, p1.z), p2.z)
        );
        Vector3 maxPt(
            std::max(std::max(p0.x, p1.x), p2.x),
            std::max(std::max(p0.y, p1.y), p2.y),
            std::max(std::max(p0.z, p1.z), p2.z)
        );
        box = AABB(minPt, maxPt);
        // FUTURE: deal with uvs
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
    int shape_id;
    Vector3 radiance;
};

using Light = std::variant<PointLight, DiffuseAreaLight>;

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
AABB& get_bbox(Shape& curr_shape);

// Create an AABB that encloses the given 2 smaller AABBs
AABB surrounding_box(AABB box0, AABB box1);

// Used ONLY in hw_2_4
// hit box => (1,1,1), no hit => (0.5, 0.5, 0.5)
Vector3 bbox_color(std::vector<AABB> bboxes, ray& localRay);
