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
struct ShapeBase {
    int material_id = -1;
    int area_light_id = -1;
};

struct Sphere : public ShapeBase {
    Vector3 position;
    Real radius;
};

struct TriangleMesh : public ShapeBase {
    std::vector<Vector3> positions;
    std::vector<Vector3i> indices;
    std::vector<Vector3> normals;
    std::vector<Vector2> uvs;
};

struct Triangle {
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

    Triangle(int face_index, TriangleMesh* mesh, int mesh_index) :
        face_id(face_index), mesh_id(mesh_index)
    {
        // get indices
        Vector3i id3 = mesh->indices[face_index];
        // call naive constructor
        Triangle(mesh->positions[id3[0]], mesh->positions[id3[1]], mesh->positions[id3[2]]);
        // write normals
        n0 = mesh->normals[id3[0]];
        n1 = mesh->normals[id3[0]];
        n2 = mesh->normals[id3[0]];
        normal = normalize(cross(e1, e2));
        // FUTURE: deal with uvs
    }
};

using Shape = std::variant<Sphere, Triangle>;

// Material property defined in material.h

struct PointLight {
    Vector3 position;
    Vector3 intensity;
};

struct DiffuseAreaLight {
    int shape_id;
    Vector3 radiance;
};

using Light = std::variant<PointLight, DiffuseAreaLight>;

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



