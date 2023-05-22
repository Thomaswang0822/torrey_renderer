#include "hw2.h"
#include "print_scene.h"

using namespace hw2;

Image3 hw_2_1(const std::vector<std::string> &params) {
    // Homework 2.1: render a single triangle and outputs
    // its barycentric coordinates.
    // We will use the following camera parameter
    // lookfrom = (0, 0,  0)
    // lookat   = (0, 0, -1)
    // up       = (0, 1,  0)
    // vfov     = 45
    // and we will parse the triangle vertices from params
    // The three vertices are stored in v0, v1, and v2 below.

    std::vector<float> tri_params;
    int spp = 16;
    for (int i = 0; i < (int)params.size(); i++) {
        if (params[i] == "-spp") {
            spp = std::stoi(params[++i]);
        } else {
            tri_params.push_back(std::stof(params[i]));
        }
    }
    double inv_spp = 1.0 / spp;

    if (tri_params.size() < 9) {
        // Not enough parameters to parse the triangle vertices.
        return Image3(0, 0);
    }

    Vector3 p0{tri_params[0], tri_params[1], tri_params[2]};
    Vector3 p1{tri_params[3], tri_params[4], tri_params[5]};
    Vector3 p2{tri_params[6], tri_params[7], tri_params[8]};
    Triangle tri(p0, p1, p2);

    Image3 img(640 /* width */, 480 /* height */);

    // Default camera described above; reuse hw1 code
    Camera cam = {
        Vector3(0, 0,  0),  // lookfrom;
        Vector3(0, 0, -1),  // lookat
        Vector3(0, 1,  0),  // up
        45.0,   // vfov
        img.width,
        img.height
    };

    ray localRay;
    Real u, v;
    Vector3 pixel_pos;
    // setup random geneator
    pcg32_state rng = init_pcg32();
    for (int y = 0; y < img.height; y++) {
        for (int x = 0; x < img.width; x++) {
            Vector3 pixel_color = {0.0, 0.0, 0.0};
            for (int s=0; s<spp; ++s) { 
                // shoot a random ray
                u = Real(x + next_pcg32_real<double>(rng)) / (img.width - 1);
                v = Real(y + next_pcg32_real<double>(rng)) / (img.height - 1);
                localRay = cam.get_ray(u, v);

                // NEW: ray triangle intersection
                Real hitDist;
                Vector3 hitPos, baryC;
                if (RayIntersectsTriangle(localRay, tri, hitDist, hitPos, baryC)) {
                    pixel_color += Vector3(baryC);
                } else{
                    pixel_color += {0.5, 0.5, 0.5};
                }
            }
            // write average color
            img(x, img.height-1 - y) = pixel_color * inv_spp;
        }
    }

    return img;
}

Image3 hw_2_2(const std::vector<std::string> &params) {
    // Homework 2.2: render a triangle mesh.
    // We will use the same camera parameter:
    // lookfrom = (0, 0,  0)
    // lookat   = (0, 0, -1)
    // up       = (0, 1,  0)
    // vfov     = 45
    // and we will use a fixed triangle mesh: a tetrahedron!
    int spp = 16;
    for (int i = 0; i < (int)params.size(); i++) {
        if (params[i] == "-spp") {
            spp = std::stoi(params[++i]);
        }
    }
    double inv_spp = 1.0 / spp;

    std::vector<Vector3> positions = {
        Vector3{ 0.0,  0.5, -2.0},
        Vector3{ 0.0, -0.3, -1.0},
        Vector3{ 1.0, -0.5, -3.0},
        Vector3{-1.0, -0.5, -3.0}
    };
    std::vector<Vector3i> indices = {
        Vector3i{0, 1, 2},
        Vector3i{0, 3, 1},
        Vector3i{0, 2, 3},
        Vector3i{1, 2, 3}
    };

    // build simple triangle mesh
    std::vector<Triangle> tris;
    Vector3 p0, p1, p2;
    for (auto i=0u; i<indices.size(); ++i) {
        p0 = positions[indices[i][0]];
        p1 = positions[indices[i][1]];
        p2 = positions[indices[i][2]];
        tris.push_back(Triangle(p0,p1,p2));
    }

    Image3 img(640 /* width */, 480 /* height */);

    // Default camera described above; reuse hw1 code
    Camera cam = {
        Vector3(0, 0,  0),  // lookfrom;
        Vector3(0, 0, -1),  // lookat
        Vector3(0, 1,  0),  // up
        45.0,   // vfov
        img.width,
        img.height
    };

    ray localRay;
    Real u, v;
    Vector3 pixel_pos;
    // setup random geneator
    pcg32_state rng = init_pcg32();
    for (int y = 0; y < img.height; y++) {
        for (int x = 0; x < img.width; x++) {
            Vector3 pixel_color = {0.0, 0.0, 0.0};
            for (int s=0; s<spp; ++s) { 
                // shoot a random ray
                u = Real(x + next_pcg32_real<double>(rng)) / (img.width - 1);
                v = Real(y + next_pcg32_real<double>(rng)) / (img.height - 1);
                localRay = cam.get_ray(u, v);

                // NEW: ray triangle-soup intersection
                Real hitDist = infinity<Real>();
                Vector3 hitPos, baryC;
                if (RayIntersectsAny_Naive(localRay, tris, hitDist, hitPos, baryC)) {
                    pixel_color += Vector3(baryC);
                } else{
                    pixel_color += {0.5, 0.5, 0.5};
                }
            }
            // write average color
            img(x, img.height-1 - y) = pixel_color * inv_spp;
        }
    }


    
    return img;
}

Image3 hw_2_3(const std::vector<std::string> &params) {
    // Homework 2.3: render a scene file provided by our parser.
    if (params.size() < 1) {
        return Image3(0, 0);
    }

    Timer timer;
    tick(timer);
    ParsedScene scene = parse_scene(params[0]);
    std::cout << "Scene parsing done. Took " << tick(timer) << " seconds." << std::endl;
    std::cout << scene << std::endl;

    Scene myScene(scene);
    std::cout << "ParsedScene Copied to myScene. Took " << 
            tick(timer) << " seconds." << std::endl;
    
    // std::cout << "myScene.shapes has length: " << myScene.shapes.size() << std::endl;
    // BEGIN: rewrite hw_1_8() code
    int spp = myScene.samples_per_pixel;
    double inv_spp = 1.0 / spp;
    Image3 img(myScene.width, myScene.height);
    Camera cam = myScene.camera;

    // setup parallel
    constexpr int tile_size = 16;
    int num_tiles_x = (img.width + tile_size - 1) / tile_size;
    int num_tiles_y = (img.height + tile_size - 1) / tile_size;
    ProgressReporter reporter(num_tiles_x * num_tiles_y);
    // almost 100% copy from https://github.com/BachiLi/lajolla_public/blob/b8ca4d02e2c7629db672d50a113c9dd04c54c906/src/render.cpp#L80
    parallel_for([&](const Vector2i &tile){
        // use scene.camera
        ray localRay;
        Real u, v;
        // cannot directly store color now
        Vector3 pixel_color;
        // setup random geneator; give it unique stream_id
        pcg32_state rng = init_pcg32(tile[1] * num_tiles_x + tile[0]);
        // start and stop indices for each tile
        int x0 = tile[0] * tile_size;
        int x1 = std::min(x0 + tile_size, img.width);
        int y0 = tile[1] * tile_size;
        int y1 = std::min(y0 + tile_size, img.height);
        for (int y = y0; y < y1; y++) {
            for (int x = x0; x < x1; x++) {
                // for each pixel, shoot may random rays thru
                pixel_color = {0.0, 0.0, 0.0};
                for (int s=0; s<spp; ++s) {    
                    // shoot a ray
                    u = Real(x + next_pcg32_real<double>(rng)) / (img.width - 1);
                    v = Real(y + next_pcg32_real<double>(rng)) / (img.height - 1);
                    localRay = cam.get_ray(u, v);
                    
                    // CHANGE: call computePixelColor() which deal with hit & no-hit
                    pixel_color += computePixelColor(myScene, localRay);
                }
                // average and write color
                img(x, img.height-1 - y) = pixel_color * inv_spp;
            }
        }
        reporter.update(1);
    }, Vector2i(num_tiles_x, num_tiles_y));
    reporter.done();
    // END: rewrite hw_1_8() code
    return img;
}

Image3 hw_2_4(const std::vector<std::string> &params) {
    // Homework 2.4: render the AABBs of the scene.
    if (params.size() < 1) {
        return Image3(0, 0);
    }

    Timer timer;
    tick(timer);
    ParsedScene scene = parse_scene(params[0]);
    std::cout << "Scene parsing done. Took " << tick(timer) << " seconds." << std::endl;
    Scene myScene(scene);
    std::cout << "ParsedScene Copied to myScene. Took " << 
            tick(timer) << " seconds." << std::endl;
    int spp = myScene.samples_per_pixel;
    double inv_spp = 1.0 / spp;
    Image3 img(myScene.width, myScene.height);
    Camera cam = myScene.camera;

    // get a vector of AABB
    std::vector<AABB> bboxes;
    for (Shape shape : myScene.shapes) {
        bboxes.push_back(bounding_box(shape));
    }

    // setup parallel
    constexpr int tile_size = 16;
    int num_tiles_x = (img.width + tile_size - 1) / tile_size;
    int num_tiles_y = (img.height + tile_size - 1) / tile_size;
    
    // almost 100% copy from https://github.com/BachiLi/lajolla_public/blob/b8ca4d02e2c7629db672d50a113c9dd04c54c906/src/render.cpp#L80
    parallel_for([&](const Vector2i &tile){
        // use scene.camera
        ray localRay;
        Real u, v;
        // cannot directly store color now
        Vector3 pixel_color;
        // setup random geneator; give it unique stream_id
        pcg32_state rng = init_pcg32(tile[1] * num_tiles_x + tile[0]);
        // start and stop indices for each tile
        int x0 = tile[0] * tile_size;
        int x1 = std::min(x0 + tile_size, img.width);
        int y0 = tile[1] * tile_size;
        int y1 = std::min(y0 + tile_size, img.height);
        for (int y = y0; y < y1; y++) {
            for (int x = x0; x < x1; x++) {
                pixel_color = {0.0, 0.0, 0.0};
                for (int s=0; s<spp; ++s) {    
                    // shoot a ray
                    u = Real(x + next_pcg32_real<double>(rng)) / (img.width - 1);
                    v = Real(y + next_pcg32_real<double>(rng)) / (img.height - 1);
                    localRay = cam.get_ray(u, v);
                    
                    // hw_2_4 UPDATE: use bbox_color()
                    pixel_color += bbox_color(bboxes, localRay);
                }
                // average and write color
                img(x, img.height-1 - y) = pixel_color * inv_spp;
            }
        }

    }, Vector2i(num_tiles_x, num_tiles_y));
    
    return img;
}

Image3 hw_2_5(const std::vector<std::string> &params) {
    // Homework 2.5: rendering with BVHs
    if (params.size() < 1) {
        return Image3(0, 0);
    }

    Timer timer;
    tick(timer);
    ParsedScene scene = parse_scene(params[0]);
    std::cout << "Scene parsing done. Took " << tick(timer) << " seconds." << std::endl;
    // std::cout << scene << std::endl;

    Scene myScene(scene);
    std::cout << "ParsedScene Copied to myScene. Took " << 
            tick(timer) << " seconds." << std::endl;

    // construct BVH tree
    pcg32_state rng_BVH = init_pcg32();
    // DEBUG NOTE: see BVH_node.h
    std::vector<Shape>& shapes = myScene.shapes;
    std::vector<std::shared_ptr<Shape>> shape_ptrs;
    for (size_t i = 0; i < shapes.size(); ++i) {
        std::shared_ptr<Shape> shape_ptr = std::make_shared<Shape>(shapes[i]);
        shape_ptrs.push_back(shape_ptr);
    }
    // manually construct BVH for each mesh
    BVH_node root(shape_ptrs, myScene, rng_BVH);
    // BVH_node root(shape_ptrs, 0, myScene.shapes.size(), rng_BVH, false);
    std::cout << "BVH tree built. Took " << 
            tick(timer) << " seconds." << std::endl;
    
    // BEGIN: rewrite hw_1_8() code
    int spp = myScene.samples_per_pixel;
    double inv_spp = 1.0 / spp;
    Image3 img(myScene.width, myScene.height);
    Camera cam = myScene.camera;

    // setup parallel
    constexpr int tile_size = 16;
    int num_tiles_x = (img.width + tile_size - 1) / tile_size;
    int num_tiles_y = (img.height + tile_size - 1) / tile_size;
    ProgressReporter reporter(num_tiles_x * num_tiles_y);
    // almost 100% copy from https://github.com/BachiLi/lajolla_public/blob/b8ca4d02e2c7629db672d50a113c9dd04c54c906/src/render.cpp#L80
    parallel_for([&](const Vector2i &tile){
        // use scene.camera
        ray localRay;
        Real u, v;
        // cannot directly store color now
        Vector3 pixel_color;
        // setup random geneator; give it unique stream_id
        pcg32_state rng = init_pcg32(tile[1] * num_tiles_x + tile[0]);
        // start and stop indices for each tile
        int x0 = tile[0] * tile_size;
        int x1 = std::min(x0 + tile_size, img.width);
        int y0 = tile[1] * tile_size;
        int y1 = std::min(y0 + tile_size, img.height);
        for (int y = y0; y < y1; y++) {
            for (int x = x0; x < x1; x++) {
                // for each pixel, shoot may random rays thru
                pixel_color = {0.0, 0.0, 0.0};
                for (int s=0; s<spp; ++s) {    
                    // shoot a ray
                    u = Real(x + next_pcg32_real<double>(rng)) / (img.width - 1);
                    v = Real(y + next_pcg32_real<double>(rng)) / (img.height - 1);
                    localRay = cam.get_ray(u, v);
                    
                    // CHANGE: call computePixelColor() which deal with hit & no-hit
                    pixel_color += BVH_PixelColor(myScene, localRay, root, rng);
                }
                // average and write color
                img(x, img.height-1 - y) = pixel_color * inv_spp;
            }
        }
        reporter.update(1);
    }, Vector2i(num_tiles_x, num_tiles_y));
    reporter.done();
    // END: rewrite hw_1_8() code
    std::cout << "Parallel Raytracing takes: " << tick(timer) << " seconds.\n ";
    return img;
}
