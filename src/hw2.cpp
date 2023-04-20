#include "hw2.h"
#include "parse_scene.h"
#include "print_scene.h"
#include "timer.h"


bool RayIntersectsTriangle(ray localRay, 
                           Triangle tri,
                           Vector3& outIntersectionPoint,
                           Vector3& baryC)
{
    const float EPSILON = 1e-7;

    Vector3 h, s, q;
    Real a, f, u, v;
    h = cross(localRay.direction(), tri.e2);
    a = dot(tri.e1, h);

    if (a > -EPSILON && a < EPSILON) {
        return false;    // This ray is parallel to this triangle.
    }
    f = 1.0 / a;
    s = localRay.origin() - tri.p0;
    u = f * dot(s, h);

    if (u < 0.0 || u > 1.0) {
        return false;   // hit point not in triangle
    }

    q = cross(s, tri.e1);
    v = f * dot(localRay.direction(), q);

    if (v < 0.0 || u + v > 1.0) {
        return false;   // hit point not in triangle
    }

    // At this stage we can compute t to find out where the intersection point is on the line.
    float t = f * dot(tri.e2, q);

    if (t > EPSILON) // ray intersection
    {
        outIntersectionPoint = localRay.at(t);
        baryC = {1.0-u-v, u, v};
        return true;
    }
    else {// This means that there is a line intersection but not a ray intersection.
        return false;
    }
}

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
        45.0   // vfov
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
                Vector3 hitPos, baryC;
                if (RayIntersectsTriangle(localRay, tri, hitPos, baryC)) {
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

    Image3 img(640 /* width */, 480 /* height */);
    
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

    return Image3(0, 0);
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
    UNUSED(scene);

    return Image3(0, 0);
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
    UNUSED(scene);

    return Image3(0, 0);
}
