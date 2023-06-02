#include "hw1.h"
#include "hw1_scenes.h"
#include "parallel.h"

using namespace hw1;

// Follow RTOW section 4.2
Image3 hw_1_1(const std::vector<std::string> &/*params*/) {
    // Homework 1.1: generate camera rays and output the ray directions
    // The camera is positioned at (0, 0, 0), facing towards (0, 0, -1),
    // with an up vector (0, 1, 0) and a vertical field of view of 90 degree.

    Image3 img(640 /* width */, 480 /* height */);

    // Camera consts -> Camera object
    Vector3 origin(0.0, 0.0, 0.0);
    Vector3 up(0.0, 1.0, 0.0);
    Camera camera = Camera(origin, Vector3(0.0, 0.0, -1.0), up, 90);

    ray localRay;
    Real u, v;
    Vector3 pixel_pos;
    for (int y = 0; y < img.height; y++) {
        // Why not here
        v = Real(y) / (img.height - 1);
        for (int x = 0; x < img.width; x++) {
            // shoot a ray
            u = Real(x) / (img.width - 1);
            localRay = camera.get_ray(u, v);
            img(x, img.height-1 - y) = localRay.dir;
        }
    }
    return img;
}

// Section 6.2
double hit_sphere(const Sphere& ball, const ray& r) {
    Vector3 oc = r.orig - ball.center;
    // a,b,c refer to those in at^2 + bt + c = 0
    auto a = length_squared(r.dir);
    auto half_b = dot(oc, r.dir);
    auto c = length_squared(oc) - ball.radius * ball.radius;
    auto discriminant = half_b*half_b - a*c;
    if (discriminant < 0) {
        // return -1.0;
        return infinity<double>();
    } else {
        // minus because we want the closer hitting point -> smaller t
        double smallerRoot = (-half_b - sqrt(discriminant) ) / a;
        double biggerRoot = (-half_b + sqrt(discriminant) ) / a;
        if (smallerRoot < 1e-4) {
            // check the larger root
            return (biggerRoot < 1e-4)? infinity<double>() : biggerRoot;
        }
        return smallerRoot;
    }

}

Image3 hw_1_2(const std::vector<std::string> &/*params*/) {
    // Homework 1.2: intersect the rays generated from hw_1_1
    // with a unit sphere located at (0, 0, -2)
    Sphere sphere = {
        Vector3(0.0, 0.0, -2.0), // center
        Real(1.0),    // unit sphere
        0       // material irrelevant for now
    };

    Image3 img(640 /* width */, 480 /* height */);

    // Camera consts -> Camera object
    Vector3 origin(0.0, 0.0, 0.0);
    Vector3 up(0.0, 1.0, 0.0);
    Camera camera = Camera(origin, sphere.center, up, 90);

    ray localRay;
    Real u, v;
    Vector3 pixel_pos;
    double hitResult; Vector3 sphereNormal;
    for (int y = 0; y < img.height; y++) {
        // Why not here
        v = Real(y) / (img.height - 1);
        for (int x = 0; x < img.width; x++) {
            // shoot a ray
            u = Real(x) / (img.width - 1);
            localRay = camera.get_ray(u, v);
            
            // try to hit the sphere
            hitResult = hit_sphere(sphere, localRay);
            if (hitResult > 1e9) {  // if hitResult == inf
                img(x, img.height-1 - y) = {0.5, 0.5, 0.5};
            } else {
                sphereNormal = normalize(localRay.at(hitResult) - sphere.center);
                img(x, img.height-1 - y) = (sphereNormal + 1.0) / 2.0;
            }
        }
    }

    return img;
}

Image3 hw_1_3(const std::vector<std::string> &params) {
    // Homework 1.3: add camera control to hw_1_2. 
    // We will use a look at transform:
    // The inputs are "lookfrom" (camera position),
    //                "lookat" (target),
    //                and the up vector
    // and the vertical field of view (in degrees).
    // If the user did not specify, fall back to the default
    // values below.
    // If you use the default values, it should render
    // the same image as hw_1_2.

    Vector3 lookfrom = Vector3{0, 0,  0};
    Vector3 lookat   = Vector3{0, 0, -2};
    Vector3 up       = Vector3{0, 1,  0};
    Real    vfov     = 90;
    for (int i = 0; i < (int)params.size(); i++) {
        if (params[i] == "-lookfrom") {
            Real x = std::stof(params[++i]);
            Real y = std::stof(params[++i]);
            Real z = std::stof(params[++i]);
            lookfrom = Vector3{x, y, z};
        } else if (params[i] == "-lookat") {
            Real x = std::stof(params[++i]);
            Real y = std::stof(params[++i]);
            Real z = std::stof(params[++i]);
            lookat = Vector3{x, y, z};
        } else if (params[i] == "-up") {
            Real x = std::stof(params[++i]);
            Real y = std::stof(params[++i]);
            Real z = std::stof(params[++i]);
            up = Vector3{x, y, z};
        } else if (params[i] == "-vfov") {
            vfov = std::stof(params[++i]);
        }
    }

    // avoid unused warnings
    UNUSED(lookfrom);
    UNUSED(lookat);
    UNUSED(up);
    UNUSED(vfov);

    Image3 img(640 /* width */, 480 /* height */);
    Real aspect_ratio = Real(img.width) / Real(img.height);
    // Just call Camera constructor
    Camera camera = Camera(lookfrom, lookat, up, vfov, aspect_ratio=aspect_ratio);
    // same sphere
    Sphere sphere = {
        Vector3(0.0, 0.0, -2.0), // center
        Real(1.0),    // unit sphere
        0       // material irrelevant for now
    };

    ray localRay;
    Real u, v;
    Vector3 pixel_pos;
    double hitResult; Vector3 sphereNormal;
    for (int y = 0; y < img.height; y++) {
        // Why not here
        v = Real(y) / (img.height - 1);
        for (int x = 0; x < img.width; x++) {
            // shoot a ray
            u = Real(x) / (img.width - 1);
            localRay = camera.get_ray(u, v);
            
            // try to hit the sphere
            hitResult = hit_sphere(sphere, localRay);
            if (hitResult > 1e9) {  // if hitResult == inf
                img(x, img.height-1 - y) = {0.5, 0.5, 0.5};
            } else {
                sphereNormal = normalize(localRay.at(hitResult) - sphere.center);
                img(x, img.height-1 - y) = (sphereNormal + 1.0) / 2.0;
            }
        }
    }
    return img;
}

Image3 hw_1_4(const std::vector<std::string> &params) {
    // Homework 1.4: render the scenes defined in hw1_scenes.h
    // output their diffuse color directly.
    if (params.size() == 0) {
        return Image3(0, 0);
    }

    int scene_id = std::stoi(params[0]);
    UNUSED(scene_id); // avoid unused warning
    // Your scene is hw1_scenes[scene_id]
    Scene scene = hw1_scenes[scene_id];


    Image3 img(640 /* width */, 480 /* height */);
    // use scene.camera
    ray localRay;
    Real u, v;
    Vector3 pixel_pos;
    double hitResult, currHit;
    int sphereId = -1; Sphere sphere;
    for (int y = 0; y < img.height; y++) {
        // Why not here
        v = Real(y) / (img.height - 1);
        for (int x = 0; x < img.width; x++) {
            // shoot a ray
            u = Real(x) / (img.width - 1);
            localRay = scene.camera.get_ray(u, v);
            
            // CHANGE: try to hit the EVERY sphere
            // and keep the nearest hit
            hitResult = infinity<double>();
            sphereId = -1;
            for (unsigned int i=0; i<scene.shapes.size(); ++i) {
                sphere = scene.shapes[i];
                currHit = hit_sphere(sphere, localRay);
                // currHit > 0 to make sure the ray doesn't go "backward"
                // This can happen when the sphere is behind 
                // or is huge and encloses the camera
                if (currHit > 0 && currHit < hitResult) {
                    hitResult = currHit;
                    sphereId = i;
                }
                
            }
            if (sphereId == -1) {
                // no hit
                img(x, img.height-1 - y) = {0.5, 0.5, 0.5};
            } else {
                img(x, img.height-1 - y) = scene.materials[scene.shapes[sphereId].material_id].color;
            }
        }
    }


    return img;
}


bool isVisible(Vector3& shadingPt, Vector3& lightPos, std::vector<Sphere> all_spheres) {
    double d = distance(shadingPt, lightPos);
    // shot ray from light to shadingPt
    ray localRay(lightPos, shadingPt, true);
    // test hitting point
    // Baseline version: traverse all spheres and test
    double hit_t = infinity<double>();
    for (Sphere sphere : all_spheres) {
        hit_t = std::min({hit_t, hit_sphere(sphere, localRay)});
    }
    // Epsilon Trick
    const double eps = 1e-4;
    return !bool(hit_t > eps && hit_t < (1-eps) * d);
}


/**
 * @brief Compute the color of a given pixel with precomputed info
 * @details point lights and Lambertian (diffuse) surface
 * 
 * @param scene: gives light info, 
 * @param sphereId: gives the hitting object and its info, like Kd
 * @param shadingPt: gives geometric info
 * 
 * @return The color resulted from ALL lights in the scene
 * 
 */
Vector3 compute_diffuse_color(Scene& scene, int sphereId, Vector3 shadingPt) {
    Vector3 result = Vector3(0.0, 0.0, 0.0);

    // sphere attributes:
    Sphere sphere = scene.shapes[sphereId];
    Vector3 Kd = scene.materials[sphere.material_id].color;

    // Pure geometric info:
    Vector3 normal = normalize(shadingPt - sphere.center);

    // attributes that are different for each light
    Vector3 l;  // normalized shadingPt to light position
    Real dsq;  // distance squared: shadingPt to light position
    for (PointLight light : scene.lights) {
        l = normalize(light.position - shadingPt);
        dsq = distance_squared(shadingPt, light.position);
        if (isVisible(shadingPt, light.position, scene.shapes)) {
            // abs(dot(normal, l)) can be replaced by using helper function below
            // (incomingRayOutside(-l, normal))? dot(normal, -l):dot(normal, -l);
            // but obviously it's unnecessary here.
            result += Kd * std::max( abs(dot(normal, l)), 0.0 ) * 
                c_INVPI * light.intensity / dsq;
        }
    }
    return result;
}


/**
 * @brief RTOW Section 6.4, Listing 16. Determine if an incoming Ray
 * to an object (sphere) is outside or inside the sphere
 * 
 * @return (bool) the incoming Ray is outside
 */
inline bool incomingRayOutside(Vector3& incomingDir, Vector3& outNormal) {
    return !bool(dot(incomingDir, outNormal) > 0.0);
}


Image3 hw_1_5(const std::vector<std::string> &params) {
    // Homework 1.5: render the scenes defined in hw1_scenes.h,
    // light them using the point lights in the scene.
    if (params.size() == 0) {
        return Image3(0, 0);
    }

    int scene_id = std::stoi(params[0]);
    UNUSED(scene_id); // avoid unused warning
    // Your scene is hw1_scenes[scene_id]
    Scene scene = hw1_scenes[scene_id];


    Image3 img(640 /* width */, 480 /* height */);
    // use scene.camera
    ray localRay;
    Real u, v;
    Vector3 pixel_pos;
    double hitResult, currHit;
    int sphereId = -1; Sphere sphere;
    for (int y = 0; y < img.height; y++) {
        // Why not here
        v = Real(y) / (img.height - 1);
        for (int x = 0; x < img.width; x++) {
            // shoot a ray
            u = Real(x) / (img.width - 1);
            localRay = scene.camera.get_ray(u, v);
            
            // CHANGE: try to hit the EVERY sphere
            // and keep the nearest hit
            hitResult = infinity<double>();
            sphereId = -1;
            for (unsigned int i=0; i<scene.shapes.size(); ++i) {
                sphere = scene.shapes[i];
                currHit = hit_sphere(sphere, localRay);
                // currHit > 0 to make sure the ray doesn't go "backward"
                // This can happen when the sphere is behind 
                // or is huge and encloses the camera
                if (currHit > 0 && currHit < hitResult) {
                    hitResult = currHit;
                    sphereId = i;
                }
                
            }
            if (sphereId == -1) {
                // no hit
                img(x, img.height-1 - y) = {0.5, 0.5, 0.5};
            } else {
                // compute color via a helper function
                img(x, img.height-1 - y) = compute_diffuse_color(
                    scene, 
                    sphereId, 
                    localRay.at(hitResult)    // hit position
                );
            }
        }
    }

    return img;
}

Image3 hw_1_6(const std::vector<std::string> &params) {
    // Homework 1.6: add antialiasing to homework 1.5
    if (params.size() == 0) {
        return Image3(0, 0);
    }

    int scene_id = 0;
    int spp = 64;
    for (int i = 0; i < (int)params.size(); i++) {
        if (params[i] == "-spp") {
            spp = std::stoi(params[++i]);
        } else {
            scene_id = std::stoi(params[i]);
        }
    }
    double inv_spp = 1.0 / spp;

    UNUSED(scene_id); // avoid unused warning
    UNUSED(spp); // avoid unused warning
    // Your scene is hw1_scenes[scene_id]
    Scene scene = hw1_scenes[scene_id];


    Image3 img(640 /* width */, 480 /* height */);
    // use scene.camera
    ray localRay;
    Real u, v;
    Vector3 pixel_pos;
    double hitResult, currHit;
    int sphereId = -1; Sphere sphere;
    // cannot directly store color now
    Vector3 pixel_color;
    // setup random geneator
    pcg32_state rng = init_pcg32();
    // std::uniform_real_distribution<double> randZeroOne(0.0, 1.0);
    // usage: randZeroOne(rng);
    for (int y = 0; y < img.height; y++) {
        for (int x = 0; x < img.width; x++) {
            // for each pixel, shoot may random rays thru
            pixel_color = {0.0, 0.0, 0.0};
            for (int s=0; s<spp; ++s) {    
                // shoot a ray
                u = Real(x + next_pcg32_real<double>(rng)) / (img.width - 1);
                v = Real(y + next_pcg32_real<double>(rng)) / (img.height - 1);
                localRay = scene.camera.get_ray(u, v);
                
                // CHANGE: try to hit the EVERY sphere
                // and keep the nearest hit
                hitResult = infinity<double>();
                sphereId = -1;
                for (unsigned int i=0; i<scene.shapes.size(); ++i) {
                    sphere = scene.shapes[i];
                    currHit = hit_sphere(sphere, localRay);
                    // currHit > 0 to make sure the ray doesn't go "backward"
                    // This can happen when the sphere is behind 
                    // or is huge and encloses the camera
                    if (currHit > 0 && currHit < hitResult) {
                        hitResult = currHit;
                        sphereId = i;
                    }
                    
                }
                if (sphereId == -1) {
                    // no hit
                    pixel_color += {0.5, 0.5, 0.5};
                } else {
                    // compute color via a helper function
                    pixel_color += compute_diffuse_color(
                        scene, 
                        sphereId, 
                        localRay.at(hitResult)    // hit position
                    );
                }
            }
            // average and write color
            img(x, img.height-1 - y) = pixel_color * inv_spp;
        }
    }

    return img;
}


/**
 * @brief mirror-only raytracer to compute the pixel color
 * 
 * @note It can handle
 * A. No hit -> (0.5, 0.5, 0.5)
 * B. hitting diffuse (Lambertian) surface -> compute_diffuse_color()
 * C. hitting mirror surface -> recursive call
 * 
 * @param scene 
 * @param localRay: can be primary ray (from camera) or reflected ray 
 * @param recDepth: capped at 20; defined in hw1.h
 * @return Vector3 
 */
Vector3 compute_pixel_color(Scene& scene, ray& localRay, unsigned int recDepth=MAX_RECURSION) {
    // Step 1: detect hit
    double hitResult = infinity<double>();
    int sphereId = -1;
    Sphere sphere;
    double currHit;
    const double eps = 1e-4;
    for (unsigned int i=0; i<scene.shapes.size(); ++i) {
        sphere = scene.shapes[i];
        currHit = hit_sphere(sphere, localRay);
        /* if (distance(localRay.orig, sphere.center) < 0.5 - 1e-6) {
            std::cout << "Create mirror ray at depth: " << recDepth << std::endl;
            std::cerr << "Ray orig: " << localRay.orig << std::endl;
            std::cerr << "***FROM: " << localRay.srcObj() 
                << " TO: " << i << std::endl;
            std::cerr << "***Origin inside sphere? " << distance(localRay.orig, sphere.center) << std::endl;
            std::cerr << "Ray dir: " << localRay.dir << std::endl;
            std::cerr << "Sphere center: " << sphere.center << std::endl;
            throw std::runtime_error("ORIGIN inside sphere");
        } */
        if (currHit > eps && currHit < hitResult) {
            hitResult = currHit;
            sphereId = i;
        }   
    }
    if (sphereId == -1) {
        // no hit
        return Vector3(0.5, 0.5, 0.5);
    }

    // Step 2: found hit -> act according to diffuse or mirror
    sphere = scene.shapes[sphereId];
    Material currMaterial = scene.materials[sphere.material_id];
    if (currMaterial.type == MaterialType::Diffuse || recDepth == 0) {
        // call helper function; defined before hw 1_5        
        return compute_diffuse_color(scene, sphereId, localRay.at(hitResult));
    } else {
        // mirror refect and recursion
        Vector3 hitPt = localRay.at(hitResult);      
        ray rayOut = mirror_ray(
            localRay,
            normalize(hitPt - sphere.center),   // normal
            hitPt
        );
        rayOut.src = sphereId; // DEBUG purpose; see ray.h      
        
        return currMaterial.color // color at current hitting pt
                * compute_pixel_color(scene, rayOut, recDepth=recDepth-1);   // element-wise mutiply
    }

    return Vector3(0.0, 0.0, 0.0);  // all-black to indicate a bug
}

Image3 hw_1_7(const std::vector<std::string> &params) {
    // Homework 1.7: add mirror materials to homework 1.6
    if (params.size() == 0) {
        return Image3(0, 0);
    }

    int scene_id = 0;
    int spp = 64;
    for (int i = 0; i < (int)params.size(); i++) {
        if (params[i] == "-spp") {
            spp = std::stoi(params[++i]);
        } else {
            scene_id = std::stoi(params[i]);
        }
    }
    double inv_spp = 1.0 / spp;

    UNUSED(scene_id); // avoid unused warning
    UNUSED(spp); // avoid unused warning
    // Your scene is hw1_scenes[scene_id]
    Scene scene = hw1_scenes[scene_id];

    Image3 img(640 /* width */, 480 /* height */);
    // use scene.camera
    ray localRay;
    Real u, v;
    // cannot directly store color now
    Vector3 pixel_color;
    // setup random geneator
    pcg32_state rng = init_pcg32();
    // std::uniform_real_distribution<double> randZeroOne(0.0, 1.0);
    // usage: randZeroOne(rng);
    for (int y = 0; y < img.height; y++) {
        for (int x = 0; x < img.width; x++) {
            // for each pixel, shoot may random rays thru
            pixel_color = {0.0, 0.0, 0.0};
            for (int s=0; s<spp; ++s) {    
                // shoot a ray
                u = Real(x + next_pcg32_real<double>(rng)) / (img.width - 1);
                v = Real(y + next_pcg32_real<double>(rng)) / (img.height - 1);
                localRay = scene.camera.get_ray(u, v);
                
                // CHANGE: call compute_pixel_color() which deal with hit & no-hit
                pixel_color += compute_pixel_color(scene, localRay);
            }
            // average and write color
            img(x, img.height-1 - y) = pixel_color * inv_spp;
        }
    }

    return img;
}


Image3 hw_1_8(const std::vector<std::string> &params) {
    // Homework 1.8: parallelize HW 1.7
    if (params.size() == 0) {
        return Image3(0, 0);
    }

    int scene_id = 0;
    int spp = 64;
    for (int i = 0; i < (int)params.size(); i++) {
        if (params[i] == "-spp") {
            spp = std::stoi(params[++i]);
        } else {
            scene_id = std::stoi(params[i]);
        }
    }
    double inv_spp = 1.0 / spp;

    UNUSED(scene_id); // avoid unused warning
    UNUSED(spp); // avoid unused warning
    // Your scene is hw1_scenes[scene_id]
    Scene scene = hw1_scenes[scene_id];

    Image3 img(1280 /* width */, 960 /* height */);

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
                // for each pixel, shoot may random rays thru
                pixel_color = {0.0, 0.0, 0.0};
                for (int s=0; s<spp; ++s) {    
                    // shoot a ray
                    u = Real(x + next_pcg32_real<double>(rng)) / (img.width - 1);
                    v = Real(y + next_pcg32_real<double>(rng)) / (img.height - 1);
                    localRay = scene.camera.get_ray(u, v);
                    
                    // CHANGE: call compute_pixel_color() which deal with hit & no-hit
                    pixel_color += compute_pixel_color(scene, localRay);
                }
                // average and write color
                img(x, img.height-1 - y) = pixel_color * inv_spp;
            }
        }

    }, Vector2i(num_tiles_x, num_tiles_y));

    return img;
}


Vector3 compute_pixel_color2(Scene& scene, ray& localRay,
    pcg32_state &rng, /* To do Schlick's approximation for reflectance check*/
    unsigned int recDepth=MAX_RECURSION) {
    // Step 1: detect hit
    double hitResult = infinity<double>();
    int sphereId = -1;
    Sphere sphere;
    double currHit;
    const double eps = 1e-4;
    for (unsigned int i=0; i<scene.shapes.size(); ++i) {
        sphere = scene.shapes[i];
        currHit = hit_sphere(sphere, localRay);
        /* if (distance(localRay.orig, sphere.center) < 0.5 - 1e-6) {
            std::cout << "Create mirror ray at depth: " << recDepth << std::endl;
            std::cerr << "Ray orig: " << localRay.orig << std::endl;
            std::cerr << "***FROM: " << localRay.srcObj() 
                << " TO: " << i << std::endl;
            std::cerr << "***Origin inside sphere? " << distance(localRay.orig, sphere.center) << std::endl;
            std::cerr << "Ray dir: " << localRay.dir << std::endl;
            std::cerr << "Sphere center: " << sphere.center << std::endl;
            throw std::runtime_error("ORIGIN inside sphere");
        } */
        if (currHit > eps && currHit < hitResult) {
            hitResult = currHit;
            sphereId = i;
        }   
    }
    if (sphereId == -1) {
        // no hit
        return Vector3(0.5, 0.5, 0.5);
    }

    // Step 2: found hit -> act according to diffuse or mirror OR glass
    sphere = scene.shapes[sphereId];
    Material currMaterial = scene.materials[sphere.material_id];
    
    if (currMaterial.type == MaterialType::Diffuse || recDepth == 0) {
        // call helper function; defined before hw 1_5        
        return compute_diffuse_color(scene, sphereId, localRay.at(hitResult));
    } else {
        Vector3 hitPt = localRay.at(hitResult);
        Vector3 sphereNormal = normalize(hitPt - sphere.center);
        // --Need to know if the ray will hit sphere from inside or outside
        bool hitFromOutside = dot(localRay.dir, sphereNormal) < 0.0;
        // --such that we compute eta_ratio in advance
        double eta_ratio = (hitFromOutside)? 
            1.0 / RefractIndices[currMaterial.type] : RefractIndices[currMaterial.type];
        // --to determine if refract is possible
        double cos_theta = fmin(dot(-localRay.dir, sphereNormal), 1.0);
        double sin_theta = sqrt(1.0 - cos_theta*cos_theta);
        bool cannot_refract = eta_ratio * sin_theta > 1.0;

        if (currMaterial.type == MaterialType::Mirror || 
            cannot_refract ||
            reflectance(cos_theta, eta_ratio) > next_pcg32_real<double>(rng)) {
            // mirror refect and recursion      
            ray rayOut = mirror_ray(
                localRay,
                sphereNormal,
                hitPt
            );
            rayOut.src = sphereId; // DEBUG purpose; see ray.h      
            
            return currMaterial.color // color at current hitting pt
                    * compute_pixel_color2(scene, rayOut, rng, recDepth=recDepth-1);   // element-wise mutiply
        } else {    // refract-able material glass or diamond
            // generate ray
            ray rayOut = refract_ray(
                localRay,
                sphereNormal,
                hitPt,
                eta_ratio
            );
            rayOut.src = sphereId; // DEBUG purpose; see ray.h
            // we doing refraction, attenuation is {1.0, 1.0, 1.0}, i.e. no weakening
            return compute_pixel_color2(scene, rayOut, rng, recDepth=recDepth-1);
        }
    }

    return Vector3(0.0, 0.0, 0.0);  // all-black to indicate a bug
}


Image3 hw_1_10(const std::vector<std::string> &params) {
    // Reuse code from 1.7: 
    // parallelization may cause trouble when passing rng around
    if (params.size() == 0) {
        return Image3(0, 0);
    }

    int scene_id = 0;
    int spp = 64;
    for (int i = 0; i < (int)params.size(); i++) {
        if (params[i] == "-spp") {
            spp = std::stoi(params[++i]);
        } else {
            scene_id = std::stoi(params[i]);
        }
    }
    double inv_spp = 1.0 / spp;

    UNUSED(scene_id); // avoid unused warning
    UNUSED(spp); // avoid unused warning
    // Your scene is hw1_scenes[scene_id]
    Scene scene = hw1_scenes[scene_id];

    Image3 img(1280, 960);
    // use scene.camera
    ray localRay;
    Real u, v;
    // cannot directly store color now
    Vector3 pixel_color;
    // setup random geneator
    pcg32_state rng = init_pcg32();
    for (int y = 0; y < img.height; y++) {
        for (int x = 0; x < img.width; x++) {
            // for each pixel, shoot may random rays thru
            pixel_color = {0.0, 0.0, 0.0};
            for (int s=0; s<spp; ++s) {    
                // shoot a ray
                u = Real(x + next_pcg32_real<double>(rng)) / (img.width - 1);
                v = Real(y + next_pcg32_real<double>(rng)) / (img.height - 1);
                localRay = scene.camera.get_ray(u, v);
                
                // CHANGE: call compute_pixel_color2() which deal with 
                // hit & no-hit
                // (if hit): reflect or refract
                pixel_color += compute_pixel_color2(scene, localRay, rng);
            }
            // average and write color
            img(x, img.height-1 - y) = pixel_color * inv_spp;
        }
    }

    return img;
}


Image3 hw_1_11(const std::vector<std::string> &params) {
    std::cout << "rendering with defocus blur" << std::endl;
    if (params.size() == 0) {
        return Image3(0, 0);
    }

    int scene_id = 0;
    int spp = 64;
    for (int i = 0; i < (int)params.size(); i++) {
        if (params[i] == "-spp") {
            spp = std::stoi(params[++i]);
        } else {
            scene_id = std::stoi(params[i]);
        }
    }
    double inv_spp = 1.0 / spp;

    UNUSED(scene_id); // avoid unused warning
    UNUSED(spp); // avoid unused warning
    Scene scene = hw1_scenes[scene_id];
    // manually hard-code a blurCamera
    blurCamera camera = blurCamera(
        Vector3{0, 0, 0},  // lookfrom
        Vector3{0, 0, -1}, // lookat
        Vector3{0, 1, 0},  // up
        45,                 // vfov
        4.0 / 3.0,   // aspect_ratio
        0.01,        // aperture; should be pretty small relative to obj size
        1.0         // dist_to_focus
    );

    Image3 img(1280 /* width */, 960 /* height */);

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
                // for each pixel, shoot may random rays thru
                pixel_color = {0.0, 0.0, 0.0};
                for (int s=0; s<spp; ++s) {    
                    // shoot a ray
                    u = Real(x + next_pcg32_real<double>(rng)) / (img.width - 1);
                    v = Real(y + next_pcg32_real<double>(rng)) / (img.height - 1);
                    localRay = camera.get_ray(u, v, rng);
                    
                    // CHANGE: call compute_pixel_color() which deal with hit & no-hit
                    pixel_color += compute_pixel_color(scene, localRay);
                }
                // average and write color
                img(x, img.height-1 - y) = pixel_color * inv_spp;
            }
        }

    }, Vector2i(num_tiles_x, num_tiles_y));

    return img;
}
