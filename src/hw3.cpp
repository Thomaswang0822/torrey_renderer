#include "hw3.h"
#include "print_scene.h"

Image3 hw_3_1(const std::vector<std::string> &params) {
    // Homework 3.1: image textures
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
    // std::vector<Shape>& shapes = myScene.shapes;
    std::vector<std::shared_ptr<Shape>> shape_ptrs;
    for (size_t i = 0; i < myScene.shapes.size(); ++i) {
        std::shared_ptr<Shape> shape_ptr = std::make_shared<Shape>(myScene.shapes[i]);
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

// Alias to hw_3_1()
// shading normal only involves a tiny fix.
// See helper.cpp L90 and Scene.h L204
Image3 hw_3_2(const std::vector<std::string> &params) {
    // Homework 3.2: shading normals
    return hw_3_1(params);
}

Image3 hw_3_3(const std::vector<std::string> &params) {
    // Homework 3.3: Fresnel
    return hw_3_1(params);
}

Image3 hw_3_4(const std::vector<std::string> &params) {
    // Homework 3.4: area lights
    return hw_3_1(params);
}
