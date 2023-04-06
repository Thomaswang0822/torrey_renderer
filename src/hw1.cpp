#include "hw1.h"
#include "hw1_scenes.h"
// #include "Camera.h"

using namespace hw1;

// Follow RTOW section 4.2
Image3 hw_1_1(const std::vector<std::string> &/*params*/) {
    // Homework 1.1: generate camera rays and output the ray directions
    // The camera is positioned at (0, 0, 0), facing towards (0, 0, -1),
    // with an up vector (0, 1, 0) and a vertical field of view of 90 degree.

    Image3 img(640 /* width */, 480 /* height */);
    const double aspect_ratio = 4.0 / 3.0;

    // Camera consts
    auto viewport_height = 2.0;
    auto viewport_width = aspect_ratio * viewport_height;
    auto focal_length = 1.0;

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
            img(x, img.height-1 - y) = localRay.direction();
        }
    }
    return img;
}

// Section 6.2
double hit_sphere(const Sphere& ball, const ray& r) {
    Vector3 oc = r.origin() - ball.center;
    // a,b,c refer to those in at^2 + bt + c = 0
    auto a = length_squared(r.direction());
    auto half_b = dot(oc, r.direction());
    auto c = length_squared(oc) - ball.radius * ball.radius;
    auto discriminant = half_b*half_b - a*c;
    if (discriminant < 0) {
        return -1.0;
    } else {
        // minus because we want the closer hitting point -> smaller t
        return (-half_b - sqrt(discriminant) ) / a;
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
    const double aspect_ratio = 4.0 / 3.0;

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
            if (abs(hitResult + 1.0) < 1e-9) {  // if hitResult == -1.0
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
            if (abs(hitResult + 1.0) < 1e-9) {  // if hitResult == -1.0
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

    Image3 img(640 /* width */, 480 /* height */);

    return img;
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

    Image3 img(640 /* width */, 480 /* height */);

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

    UNUSED(scene_id); // avoid unused warning
    UNUSED(spp); // avoid unused warning
    // Your scene is hw1_scenes[scene_id]

    Image3 img(160 /* width */, 120 /* height */);

    return img;
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

    UNUSED(scene_id); // avoid unused warning
    UNUSED(spp); // avoid unused warning
    // Your scene is hw1_scenes[scene_id]

    Image3 img(640 /* width */, 480 /* height */);

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

    UNUSED(scene_id); // avoid unused warning
    UNUSED(spp); // avoid unused warning
    // Your scene is hw1_scenes[scene_id]

    Image3 img(1280 /* width */, 960 /* height */);

    return img;
}
