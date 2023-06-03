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

    /**
     * @note In ParsedScene, shapes contains Sphere and TriangleMesh;
     * here shapes contains Sphere and Triangle. So we need a way to map
     * shape_id from the ParsedScene context to that in the Scene context.
     * 
     * if ParsedScene.shapes looks like: [sph, [3], sph],
     * this map should look like [0, 1, 4]
     */
    std::vector<int> shape_id_map;
    int currIdx = 0;

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
            shape_id_map.push_back(currIdx);
            currIdx++;
        } else if (auto *parsed_mesh = get_if<ParsedTriangleMesh>(&parsed_shape)) {
            meshes[tri_mesh_count] = TriangleMesh(*parsed_mesh);
            // Extract all the individual triangles
            int nTri = meshes[tri_mesh_count].size;
            assert (nTri > 0);
            Real totalArea = 0.0;
            vector<Real> cdf(nTri+1, 0.0);  // has an extra 0 at the front
            for (int face_index = 0; face_index < nTri; face_index++) {
                Triangle currTri(face_index, &meshes[tri_mesh_count], tri_mesh_count);
                // cout << "area of this triangle: " << currTri.area << endl;
                totalArea += currTri.area;
                cdf[face_index+1] = totalArea;
                shapes.push_back(currTri);
            }
            // normalize the accumulated area and write to the mesh
            std::transform(cdf.begin(), cdf.end(), cdf.begin(), 
                std::bind1st(std::multiplies<double>(), 1.0 / totalArea)
            );
            cdf.back() = 1.0;  // avoid 0-probability numerical issue
            meshes[tri_mesh_count].areaCDF = cdf;
            meshes[tri_mesh_count].totalArea = totalArea;

            tri_mesh_count++;
            shape_id_map.push_back(currIdx);
            currIdx += nTri;
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
        } else if (auto *phong = get_if<ParsedPhong>(&parsed_mat)) {
            if (get_if<Vector3>(&phong->reflectance)) {
                materials.push_back(
                    Phong{get<Vector3>(phong->reflectance), phong->exponent}
                );
            }
            else if (get_if<ParsedImageTexture>(&phong->reflectance)) {
                materials.push_back(
                    Phong{get<ParsedImageTexture>(phong->reflectance), phong->exponent}
                );
            }
            else {
                Error("Phong reflectance: not Vector3 or ImageTexture.");
            }
        } else if (auto *blph = get_if<ParsedBlinnPhong>(&parsed_mat)) {
            if (get_if<Vector3>(&blph->reflectance)) {
                materials.push_back(
                    BlinnPhong{get<Vector3>(blph->reflectance), blph->exponent}
                );
            }
            else if (get_if<ParsedImageTexture>(&blph->reflectance)) {
                materials.push_back(
                    BlinnPhong{get<ParsedImageTexture>(blph->reflectance), blph->exponent}
                );
            }
            else {
                Error("BlinnPhong reflectance: not Vector3 or ImageTexture.");
            }
        }
        else if (auto *blph_m = get_if<ParsedBlinnPhongMicrofacet>(&parsed_mat)) {
            if (get_if<Vector3>(&blph_m->reflectance)) {
                materials.push_back(
                    Microfacet{get<Vector3>(blph_m->reflectance), blph_m->exponent}
                );
            }
            else if (get_if<ParsedImageTexture>(&blph_m->reflectance)) {
                materials.push_back(
                    Microfacet{get<ParsedImageTexture>(blph_m->reflectance), 
                        blph_m->exponent}
                );
            }
            else {
                Error("Microfacet reflectance: not Vector3 or ImageTexture.");
            }
        }
        
        else {
            Error("Material not implemented yet.");
        }
    }
    // Copy the lights
    for (const ParsedLight &parsed_light : scene.lights) {
        // HW3 UPDATE: Deal with area light
        if (get_if<ParsedPointLight>(&parsed_light)) {
            ParsedPointLight point_light = get<ParsedPointLight>(parsed_light);
            lights.push_back(PointLight{point_light.position, point_light.intensity});
        } else if (get_if<ParsedDiffuseAreaLight>(&parsed_light)) {
            ParsedDiffuseAreaLight area_light = get<ParsedDiffuseAreaLight>(parsed_light);
            // see: shape_id_map
            int primId = shape_id_map[area_light.shape_id];
            lights.push_back(DiffuseAreaLight{primId, area_light.radiance});
        }
            
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
        Vector3 minPt = min(tri->p0, min(tri->p1, tri->p2));
        Vector3 maxPt = max(tri->p0, max(tri->p1, tri->p2));
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
