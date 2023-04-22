#include "Scene.h"

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
        if (std::get_if<ParsedTriangleMesh>(&parsed_shape)) {
            tri_mesh_count++;
        }
    }
    meshes.resize(tri_mesh_count);

    // Extract the shapes
    tri_mesh_count = 0;
    for (int i = 0; i < (int)scene.shapes.size(); i++) {
        const ParsedShape &parsed_shape = scene.shapes[i];
        if (auto *sph = std::get_if<ParsedSphere>(&parsed_shape)) {
            shapes.push_back(
                Sphere{
                    {sph->material_id, sph->area_light_id},
                    sph->position, sph->radius
                }
            );
        } else if (auto *parsed_mesh = std::get_if<ParsedTriangleMesh>(&parsed_shape)) {
            meshes[tri_mesh_count] = TriangleMesh{
                {parsed_mesh->material_id, parsed_mesh->area_light_id},
                parsed_mesh->positions, parsed_mesh->indices,
                parsed_mesh->normals, parsed_mesh->uvs};
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
        if (auto *diffuse = std::get_if<ParsedDiffuse>(&parsed_mat)) {
            // We assume the reflectance is always RGB for now.
            materials.push_back(Diffuse{std::get<Vector3>(diffuse->reflectance)});
        } else if (auto *mirror = std::get_if<ParsedMirror>(&parsed_mat)) {
            // We assume the reflectance is always RGB for now.
            materials.push_back(Mirror{std::get<Vector3>(mirror->reflectance)});
        } else {
            assert(false);
        }
    }

    // Copy the lights
    for (const ParsedLight &parsed_light : scene.lights) {
        // We assume all lights are point lights for now.
        ParsedPointLight point_light = std::get<ParsedPointLight>(parsed_light);
        // lights.push_back(PointLight{point_light.intensity, point_light.position});
        // why is it swapped?
        lights.push_back(PointLight{point_light.position, point_light.intensity});
    }
}