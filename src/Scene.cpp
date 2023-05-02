#include "Scene.h"

using namespace std;

// constructor
Scene::Scene(const ParsedScene &scene) :
        camera(scene.camera),
        width(scene.camera.width),
        height(scene.camera.height),
        background_color(scene.background_color),
        samples_per_pixel(scene.samples_per_pixel) {
    // Extract triangle meshes from the parsed scene.
    int tri_mesh_count = 0;
    for (const ParsedShape &parsed_shape : scene.shapes) {
        if (get_if<ParsedTriangleMesh>(&parsed_shape)) {
            tri_mesh_count++;
        }
    }
    meshes.resize(tri_mesh_count);

    // Extract the shapes
    tri_mesh_count = 0;
    for (int i = 0; i < (int)scene.shapes.size(); i++) {
        const ParsedShape &parsed_shape = scene.shapes[i];
        if (auto *sph = get_if<ParsedSphere>(&parsed_shape)) {
            shapes.push_back(
                Sphere{
                    sph->material_id, sph->area_light_id,
                    sph->position, sph->radius
                }
            );
        } else if (auto *parsed_mesh = get_if<ParsedTriangleMesh>(&parsed_shape)) {
            meshes[tri_mesh_count] = TriangleMesh{
                {parsed_mesh->material_id, parsed_mesh->area_light_id},
                parsed_mesh->positions, 
                parsed_mesh->indices,
                parsed_mesh->normals, 
                parsed_mesh->uvs
            };
            // Extract all the individual triangles
            for (int face_index = 0; face_index < (int)parsed_mesh->indices.size(); face_index++) {
                shapes.push_back(Triangle{face_index, &meshes[tri_mesh_count], tri_mesh_count});
            }
            tri_mesh_count++;
        } else {
            assert(false);
        }
    }

    // Copy the materials
    for (const ParsedMaterial &parsed_mat : scene.materials) {
        if (auto *diffuse = get_if<ParsedDiffuse>(&parsed_mat)) {
            // We assume the reflectance is always RGB for now.
            materials.push_back(Diffuse{get<Vector3>(diffuse->reflectance)});
        } else if (auto *mirror = get_if<ParsedMirror>(&parsed_mat)) {
            // We assume the reflectance is always RGB for now.
            materials.push_back(Mirror{get<Vector3>(mirror->reflectance)});
        } else {
            assert(false);
        }
    }

    // Copy the lights
    for (const ParsedLight &parsed_light : scene.lights) {
        // We assume all lights are point lights for now.
        ParsedPointLight point_light = get<ParsedPointLight>(parsed_light);
        // lights.push_back(PointLight{point_light.intensity, point_light.position});
        // why is it swapped?
        lights.push_back(PointLight{point_light.position, point_light.intensity});
    }
}

// AABB-related helper functions
// @MOTIONBLUR
AABB bounding_box(Shape curr_shape) {
    if (Sphere *sph = get_if<Sphere>(&curr_shape)) {
        // Vector3 - Real has been overloaded
        return AABB(
            sph->position - sph->radius,
            sph->position + sph->radius,
            sph->delta_pos  // change in position over time0 to time1
        );
    } else if (Triangle *tri = get_if<Triangle>(&curr_shape)) {
        // these 2 are static position at time0
        Vector3 minPt = min(tri->p0, min(tri->p1, tri->p2));
        Vector3 maxPt = max(tri->p0, max(tri->p1, tri->p2));
        return AABB(minPt, maxPt, tri->delta_pos);
    } else {
        assert(false);
    }
}


AABB& get_bbox(shared_ptr<Shape> curr_shape) {
    if (Sphere *sph = get_if<Sphere>(curr_shape.get())) {
        // Vector3 - Real has been overloaded
        return sph->box;
    } else if (Triangle *tri = get_if<Triangle>(curr_shape.get())) {
        return tri->box;
    } else {
        assert(false);
    }
}


AABB surrounding_box(AABB box0, AABB box1) {
    Vector3 small = min(box0.minimum, box1.minimum) ;
    Vector3 big= max(box0.maximum, box1.maximum);

    return AABB(small, big);
}


Vector3 bbox_color(vector<AABB> bboxes, ray& localRay) {
    for (AABB bbox : bboxes) {
        if (bbox.hit(localRay, 0.0, infinity<double>())) {
            return Vector3(1.0, 1.0, 1.0);
        }
    }
    return Vector3(0.5, 0.5, 0.5);
}