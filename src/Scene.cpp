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
            Error("Not Sphere or TriangleMesh.");
        }
    }

    // Copy the materials
    // HW3 UPDATE: now a Color can be Vector3 or struct ImageTexture
    for (const ParsedMaterial &parsed_mat : scene.materials) {
        if (auto *diffuse = get_if<ParsedDiffuse>(&parsed_mat)) {
            if (get_if<Vector3>(&diffuse->reflectance)) {
                materials.push_back(Diffuse{get<Vector3>(diffuse->reflectance)});
            }
            else if (get_if<ParsedImageTexture>(&diffuse->reflectance)) {
                materials.push_back(Diffuse{get<ParsedImageTexture>(diffuse->reflectance)});
            }
            else {
                Error("Diffuse: not Vector3 or ImageTexture.");
            }    
        } else if (auto *mirror = get_if<ParsedMirror>(&parsed_mat)) {
            if (get_if<Vector3>(&mirror->reflectance)) {
                materials.push_back(Mirror{get<Vector3>(mirror->reflectance)});
            }
            else if (get_if<ParsedImageTexture>(&mirror->reflectance)) {
                materials.push_back(Mirror{get<ParsedImageTexture>(mirror->reflectance)});
            }
            else {
                Error("Mirror: not Vector3 or ImageTexture.");
            }
        } else if (auto *plastic = get_if<ParsedPlastic>(&parsed_mat)) {
            if (get_if<Vector3>(&plastic->reflectance)) {
                materials.push_back(
                    Plastic{plastic->eta, get<Vector3>(plastic->reflectance)}
                );
            }
            else if (get_if<ParsedImageTexture>(&plastic->reflectance)) {
                materials.push_back(
                    Plastic{plastic->eta, get<ParsedImageTexture>(plastic->reflectance)}
                );
            }
            else {
                Error("Plastic: not Vector3 or ImageTexture.");
            }
        }
        
        else {
            Error("Not Diffuse or Mirror or Plastic material.");
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
AABB bounding_box(Shape curr_shape) {
    if (Sphere *sph = get_if<Sphere>(&curr_shape)) {
        // Vector3 - Real has been overloaded
        return AABB(
            sph->position - sph->radius,
            sph->position + sph->radius
        );
    } else if (Triangle *tri = get_if<Triangle>(&curr_shape)) {
        Vector3 minPt(
            std::min(std::min(tri->p0.x, tri->p1.x), tri->p2.x),
            std::min(std::min(tri->p0.y, tri->p1.y), tri->p2.y),
            std::min(std::min(tri->p0.z, tri->p1.z), tri->p2.z)
        );
        Vector3 maxPt(
            std::max(std::max(tri->p0.x, tri->p1.x), tri->p2.x),
            std::max(std::max(tri->p0.y, tri->p1.y), tri->p2.y),
            std::max(std::max(tri->p0.z, tri->p1.z), tri->p2.z)
        );
        return AABB(minPt, maxPt);
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
    Vector3 small(
        fmin(box0.minimum.x, box1.minimum.x),
        fmin(box0.minimum.y, box1.minimum.y),
        fmin(box0.minimum.z, box1.minimum.z)
    );

    Vector3 big(
        fmax(box0.maximum.x, box1.maximum.x),
        fmax(box0.maximum.y, box1.maximum.y),
        fmax(box0.maximum.z, box1.maximum.z)
    );

    return AABB(small, big);
}
