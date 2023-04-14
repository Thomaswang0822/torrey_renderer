#pragma once

#include <unordered_map>
#include "pcg.h"

namespace hw1
{

    enum class MaterialType
    {
        Diffuse,
        Mirror,
        Glass,   // EC part
        Diamond
    };

    std::unordered_map<MaterialType, double> RefractIndices = {
        // place holders (will never be used) to enable precomputing eta_ratio;
        // see compute_pixel_color2()
        {MaterialType::Diffuse, 1.0},
        {MaterialType::Mirror, 1.0},
        // truly refract-able
        {MaterialType::Glass, 1.5},
        {MaterialType::Diamond, 2.4}
    };

    /**
     * @brief Stick to design from RTOW, sec 11.2, Listing 64
     *
     * @note Store only 4 variables. May increase if need later.
     *
     */
    struct Camera
    {
        Vector3 origin;
        // Vector3 lookat;
        // Vector3 up;
        // Real vfov;
        Vector3 horizontal, vertical;
        Vector3 lower_left_corner;

        Camera(
            Vector3 lookfrom,
            Vector3 lookat,
            Vector3 vup,
            double vfov, // vertical field-of-view in degrees
            // give it a default value s.t. prev constructor still works.
            double aspect_ratio = 4.0 / 3.0) : // origin(lookfrom), lookat(lookat), up(vup), vfov(vfov)
                                               origin(lookfrom)
        {
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

        ray get_ray(double offset_u, double offset_v) const
        {
            return ray(
                origin,
                lower_left_corner + offset_u * horizontal + offset_v * vertical - origin);
        }
    };

    // Camera with more features to implement defocus blur
    struct blurCamera : public Camera {
        // additional varaibles
        Vector3 u, v, w;
        double lens_radius;

        // overload constructor
        blurCamera(
            Vector3 lookfrom, Vector3 lookat, Vector3 vup, double vfov,  // old args
            double aspect_ratio,    // no default
            double aperture, double focus_dist   // new args
        ) : Camera(lookfrom, lookat, vup, vfov, aspect_ratio=aspect_ratio)
        {
            auto theta = radians(vfov);
            auto h = tan(theta / 2);
            auto viewport_height = 2.0 * h;
            auto viewport_width = aspect_ratio * viewport_height;

            w = normalize(lookfrom - lookat);
            u = normalize(cross(vup, w));
            v = cross(w, u);

            origin = lookfrom;
            horizontal = focus_dist * viewport_width * u;
            vertical = focus_dist * viewport_height * v;
            lower_left_corner = origin - horizontal/2.0 - vertical/2.0 - focus_dist*w;

            lens_radius = aperture / 2;
        }

        Vector3 random_in_unit_disk(pcg32_state &rng) {
            // improve: there is a more robus way to generate 
            // truly uniformly random point on a disk
            // https://stackoverflow.com/a/50746409
            double r = sqrt(next_pcg32_real<double>(rng));
            double theta = next_pcg32_real<double>(rng) * c_TWOPI;

            return Vector3(
                r * cos(theta),
                r * sin(theta),
                0.0
            );
        }

        ray get_ray(double s, double t, pcg32_state &rng) {
            Vector3 rd = lens_radius * random_in_unit_disk(rng);
            Vector3 offset = u * rd.x + v * rd.y;

            return ray(
                origin + offset,
                lower_left_corner + s*horizontal + t*vertical - origin - offset
            );
        }
    };

    struct Sphere
    {
        Vector3 center;
        Real radius;
        int material_id;
    };

    struct PointLight
    {
        Vector3 intensity;
        Vector3 position;
    };

    struct Material
    {
        MaterialType type;
        Vector3 color;
    };

    struct Scene
    {
        Camera camera;
        std::vector<Sphere> shapes;
        std::vector<Material> materials;
        std::vector<PointLight> lights;
    };

    Scene hw1_scene_0{
        Camera{
            Vector3{0, 0, 0},  // lookfrom
            Vector3{0, 0, -1}, // lookat
            Vector3{0, 1, 0},  // up
            45                 // vfov
        },
        std::vector<Sphere>{
            {Vector3{0.0, 0.0, -3.0}, 0.5, 0},
            {Vector3{0.0, -100.5, 3.0}, 100.0, 1}},
        std::vector<Material>{
            {MaterialType::Diffuse, Vector3{0.75, 0.25, 0.25}},
            {MaterialType::Diffuse, Vector3{0.25, 0.25, 0.75}}},
        std::vector<PointLight>{
            {Vector3{100, 100, 100}, Vector3{5, 5, -2}}}};

    Scene hw1_scene_1{
        Camera{
            Vector3{0, 0, 0},  // lookfrom
            Vector3{0, 0, -1}, // lookat
            Vector3{0, 1, 0},  // up
            45                 // vfov
        },
        std::vector<Sphere>{
            {Vector3{0.0, -100.5, -3.0}, 100.0, 0},
            {Vector3{0.0, 0.0, -3.0}, 0.5, 1},
            {Vector3{1.0, 0.0, -3.0}, 0.5, 2},
            {Vector3{-1.0, 0.0, -3.0}, 0.5, 3}},
        std::vector<Material>{
            {MaterialType::Diffuse, Vector3{0.80, 0.80, 0.20}},
            {MaterialType::Diffuse, Vector3{0.75, 0.25, 0.25}},
            {MaterialType::Mirror, Vector3{0.75, 0.25, 0.75}},
            {MaterialType::Mirror, Vector3{0.25, 0.75, 0.75}},
        },
        std::vector<PointLight>{
            {Vector3{100, 100, 100}, Vector3{5, 5, 2}},
            {Vector3{10, 10, 10}, Vector3{-5, 5, 1}},
            {Vector3{2, 2, 2}, Vector3{0, 5, -5}}}};

    Scene hw1_scene_2{
        Camera{
            Vector3{0, 0, 0},  // lookfrom
            Vector3{0, 0, -1}, // lookat
            Vector3{0, 1, 0},  // up
            45                 // vfov
        },
        std::vector<Sphere>{
            {Vector3{-0.75, 0.0, -4.0}, Real(1), 0},
            {Vector3{1.0, 0.0, -13.0}, Real(7.5), 1},
            {Vector3{0.5, 0.0, -3.0}, Real(0.25), 2},
        },
        std::vector<Material>{
            {MaterialType::Mirror, Vector3{0.75, 0.75, 0.25}},
            {MaterialType::Diffuse, Vector3{0.25, 0.75, 0.75}},
            {MaterialType::Mirror, Vector3{0.75, 0.25, 0.75}}},
        std::vector<PointLight>{
            {Vector3{100, 50, 50}, Vector3{1, 0, 10}},
            {Vector3{50, 50, 100}, Vector3{-1, 0, 10}},
        }};

    Scene hw1_scene_3{
        Camera{
            Vector3{0, 0, 0},  // lookfrom
            Vector3{0, 0, -1}, // lookat
            Vector3{0, 1, 0},  // up
            45                 // vfov
        },
        std::vector<Sphere>{
            {Vector3{0.0, 0.0, 0.0}, Real(100), 0},
            {Vector3{-0.35, 0.35, -3.5}, Real(0.25), 1},
            {Vector3{0.35, 0.35, -2.5}, Real(0.35), 2},
            {Vector3{0.35, -0.35, -2.0}, Real(0.3), 3},
            {Vector3{-0.35, -0.35, -4.0}, Real(0.325), 4},
            {Vector3{-1.5, 0.0, -3.0}, Real(0.5), 5},
            {Vector3{1.5, 0.0, -3.0}, Real(0.5), 6},
            {Vector3{10.0, 0.0, -3.0}, Real(0.5), 7},
            {Vector3{-10.0, 0.0, -3.0}, Real(0.5), 7},
        },
        std::vector<Material>{
            {MaterialType::Diffuse, Vector3{0.5, 0.25, 0.25}},
            {MaterialType::Diffuse, Vector3{0.25, 0.5, 0.75}},
            {MaterialType::Diffuse, Vector3{0.75, 0.5, 0.25}},
            {MaterialType::Mirror, Vector3{0.25, 0.75, 0.5}},
            {MaterialType::Diffuse, Vector3{0.5, 0.75, 0.5}},
            {MaterialType::Mirror, Vector3{0.5, 0.5, 0.75}},
            {MaterialType::Diffuse, Vector3{0.5, 0.5, 0.75}},
            {MaterialType::Diffuse, Vector3{0.75, 0.75, 0.75}}},
        std::vector<PointLight>{
            {Vector3{10, 10, 10}, Vector3{0, 0, 0}},
            {Vector3{0.5, 0.5, 0.5}, Vector3{-0.4, 0.5, -3.0}},
            {Vector3{10000, 10000, 10000}, Vector3{0, 0, 90}},
        }};

    Scene hw1_scene_4{
        Camera{
            Vector3{0, 0, 0},  // lookfrom
            Vector3{0, 0, -1}, // lookat
            Vector3{0, 1, 0},  // up
            45                 // vfov
        },
        std::vector<Sphere>{
            {Vector3{0.781, 2.293, -4.602}, 0.659, 0},
            {Vector3{-1.975, -1.115, -5.906}, 0.591, 1},
            {Vector3{-0.900, -0.518, -4.741}, 0.632, 2},
            {Vector3{-2.281, 0.900, -4.271}, 0.392, 3},
            {Vector3{0.309, 2.047, -6.365}, 0.550, 4},
            {Vector3{-1.281, 2.314, -4.383}, 0.415, 5},
            {Vector3{0.229, -0.093, -3.150}, 0.331, 6},
            {Vector3{1.400, 1.793, -3.364}, 0.322, 7},
            {Vector3{-0.795, -2.459, -6.424}, 0.563, 8},
            {Vector3{-1.045, -1.886, -4.895}, 0.418, 9},
            {Vector3{2.369, -0.402, -5.811}, 0.510, 10},
            {Vector3{-0.514, 2.270, -2.799}, 0.635, 11},
            {Vector3{0.669, 1.321, -7.112}, 0.300, 12},
            {Vector3{0.227, 1.167, -5.870}, 0.299, 13},
            {Vector3{1.320, 0.086, -7.343}, 0.510, 14},
            {Vector3{-1.410, -1.046, -2.946}, 0.280, 15},
            {Vector3{1.742, -1.488, -6.187}, 0.517, 16},
            {Vector3{-1.867, 0.315, -4.901}, 0.747, 17},
            {Vector3{-0.269, 0.208, -4.738}, 0.431, 18},
            {Vector3{0.966, 0.860, -5.417}, 0.290, 19},
            {Vector3{-2.345, 1.133, -5.147}, 0.298, 20},
            {Vector3{1.924, 1.527, -6.724}, 0.510, 21},
            {Vector3{-0.418, -1.651, -2.595}, 0.594, 22},
            {Vector3{-2.344, -0.415, -3.685}, 0.617, 23},
            {Vector3{0.238, 0.990, -3.018}, 0.306, 24},
            {Vector3{-1.016, -2.252, -2.555}, 0.392, 25},
            {Vector3{1.761, 1.176, -5.583}, 0.407, 26},
            {Vector3{1.995, 2.143, -2.525}, 0.296, 27},
            {Vector3{1.645, -1.920, -3.988}, 0.361, 28},
            {Vector3{0.189, -0.262, -4.485}, 0.298, 29}},
        std::vector<Material>{
            {MaterialType::Diffuse, Vector3{0.020, 0.660, 0.021}},
            {MaterialType::Diffuse, Vector3{0.823, 0.830, 0.703}},
            {MaterialType::Diffuse, Vector3{0.471, 0.540, 0.414}},
            {MaterialType::Mirror, Vector3{0.997, 0.048, 0.431}},
            {MaterialType::Diffuse, Vector3{0.253, 0.089, 0.712}},
            {MaterialType::Diffuse, Vector3{0.664, 0.884, 0.069}},
            {MaterialType::Diffuse, Vector3{0.399, 0.475, 0.090}},
            {MaterialType::Diffuse, Vector3{0.360, 0.298, 0.956}},
            {MaterialType::Diffuse, Vector3{0.147, 0.115, 0.440}},
            {MaterialType::Diffuse, Vector3{0.881, 0.312, 0.609}},
            {MaterialType::Diffuse, Vector3{0.946, 0.094, 0.617}},
            {MaterialType::Mirror, Vector3{0.649, 0.847, 0.018}},
            {MaterialType::Diffuse, Vector3{0.994, 0.240, 0.637}},
            {MaterialType::Diffuse, Vector3{0.228, 0.861, 0.613}},
            {MaterialType::Diffuse, Vector3{0.442, 0.546, 0.580}},
            {MaterialType::Diffuse, Vector3{0.122, 0.874, 0.081}},
            {MaterialType::Diffuse, Vector3{0.954, 0.575, 0.910}},
            {MaterialType::Diffuse, Vector3{0.601, 0.420, 0.757}},
            {MaterialType::Diffuse, Vector3{0.340, 0.136, 0.233}},
            {MaterialType::Diffuse, Vector3{0.227, 0.570, 0.241}},
            {MaterialType::Mirror, Vector3{0.906, 0.774, 0.042}},
            {MaterialType::Mirror, Vector3{0.287, 0.709, 0.301}},
            {MaterialType::Diffuse, Vector3{0.895, 0.787, 0.824}},
            {MaterialType::Mirror, Vector3{0.011, 0.395, 0.117}},
            {MaterialType::Mirror, Vector3{0.781, 0.390, 0.375}},
            {MaterialType::Mirror, Vector3{0.155, 0.873, 0.695}},
            {MaterialType::Mirror, Vector3{0.276, 0.751, 0.104}},
            {MaterialType::Diffuse, Vector3{0.503, 0.465, 0.232}},
            {MaterialType::Diffuse, Vector3{0.264, 0.794, 0.280}},
            {MaterialType::Mirror, Vector3{0.036, 0.548, 0.363}}},
        std::vector<PointLight>{
            {Vector3{10, 10, 10}, Vector3{0, 0, 0}},
            {Vector3{50, 5, 5}, Vector3{5, 5, -5}},
        }};

    // Build my own scene
    Scene hw1_scene_5{
        Camera{
            Vector3{0, 0, 0},  // lookfrom
            Vector3{0, 2, -1}, // lookat
            Vector3{0, 1, 0},  // up
            120                 // vfov
        },
        std::vector<Sphere>{
            {Vector3{0.000, 3.000, 0.000}, 0.500, 0},
            {Vector3{0.585, 2.942, 0.000}, 0.500, 1},
            {Vector3{1.148, 2.772, 0.000}, 0.500, 2},
            {Vector3{1.667, 2.494, 0.000}, 0.500, 3},
            {Vector3{2.121, 2.121, 0.000}, 0.500, 4},
            {Vector3{2.494, 1.667, 0.000}, 0.500, 5},
            {Vector3{2.772, 1.148, 0.000}, 0.500, 6},
            {Vector3{2.942, 0.585, 0.000}, 0.500, 7},
            {Vector3{3.000, 0.000, 0.000}, 0.500, 8},
            {Vector3{0.000, 3.000, 0.000}, 0.500, 9},
            {Vector3{0.546, 2.942, 0.211}, 0.500, 10},
            {Vector3{1.071, 2.772, 0.415}, 0.500, 11},
            {Vector3{1.554, 2.494, 0.602}, 0.500, 12},
            {Vector3{1.978, 2.121, 0.766}, 0.500, 13},
            {Vector3{2.326, 1.667, 0.901}, 0.500, 14},
            {Vector3{2.584, 1.148, 1.001}, 0.500, 15},
            {Vector3{2.744, 0.585, 1.063}, 0.500, 16},
            {Vector3{2.797, 0.000, 1.084}, 0.500, 17},
            {Vector3{0.000, 3.000, 0.000}, 0.500, 18},
            {Vector3{0.433, 2.942, 0.394}, 0.500, 19},
            {Vector3{0.848, 2.772, 0.773}, 0.500, 20},
            {Vector3{1.232, 2.494, 1.123}, 0.500, 21},
            {Vector3{1.568, 2.121, 1.429}, 0.500, 22},
            {Vector3{1.843, 1.667, 1.680}, 0.500, 23},
            {Vector3{2.048, 1.148, 1.867}, 0.500, 24},
            {Vector3{2.174, 0.585, 1.982}, 0.500, 25},
            {Vector3{2.217, 0.000, 2.021}, 0.500, 26},
            {Vector3{0.000, 3.000, 0.000}, 0.500, 27},
            {Vector3{0.261, 2.942, 0.524}, 0.500, 28},
            {Vector3{0.512, 2.772, 1.028}, 0.500, 29},
            {Vector3{0.743, 2.494, 1.492}, 0.500, 30},
            {Vector3{0.946, 2.121, 1.899}, 0.500, 31},
            {Vector3{1.112, 1.667, 2.233}, 0.500, 32},
            {Vector3{1.235, 1.148, 2.481}, 0.500, 33},
            {Vector3{1.312, 0.585, 2.634}, 0.500, 34},
            {Vector3{1.337, 0.000, 2.685}, 0.500, 35},
            {Vector3{0.000, 3.000, 0.000}, 0.500, 36},
            {Vector3{0.054, 2.942, 0.583}, 0.500, 37},
            {Vector3{0.106, 2.772, 1.143}, 0.500, 38},
            {Vector3{0.154, 2.494, 1.660}, 0.500, 39},
            {Vector3{0.196, 2.121, 2.112}, 0.500, 40},
            {Vector3{0.230, 1.667, 2.484}, 0.500, 41},
            {Vector3{0.256, 1.148, 2.760}, 0.500, 42},
            {Vector3{0.271, 0.585, 2.930}, 0.500, 43},
            {Vector3{0.277, 0.000, 2.987}, 0.500, 44},
            {Vector3{-0.000, 3.000, 0.000}, 0.500, 45},
            {Vector3{-0.160, 2.942, 0.563}, 0.500, 46},
            {Vector3{-0.314, 2.772, 1.104}, 0.500, 47},
            {Vector3{-0.456, 2.494, 1.603}, 0.500, 48},
            {Vector3{-0.581, 2.121, 2.040}, 0.500, 49},
            {Vector3{-0.683, 1.667, 2.399}, 0.500, 50},
            {Vector3{-0.758, 1.148, 2.666}, 0.500, 51},
            {Vector3{-0.805, 0.585, 2.830}, 0.500, 52},
            {Vector3{-0.821, 0.000, 2.885}, 0.500, 53},
            {Vector3{-0.000, 3.000, 0.000}, 0.500, 54},
            {Vector3{-0.353, 2.942, 0.467}, 0.500, 55},
            {Vector3{-0.692, 2.772, 0.916}, 0.500, 56},
            {Vector3{-1.004, 2.494, 1.330}, 0.500, 57},
            {Vector3{-1.278, 2.121, 1.693}, 0.500, 58},
            {Vector3{-1.503, 1.667, 1.991}, 0.500, 59},
            {Vector3{-1.670, 1.148, 2.212}, 0.500, 60},
            {Vector3{-1.773, 0.585, 2.348}, 0.500, 61},
            {Vector3{-1.808, 0.000, 2.394}, 0.500, 62},
            {Vector3{-0.000, 3.000, 0.000}, 0.500, 63},
            {Vector3{-0.498, 2.942, 0.308}, 0.500, 64},
            {Vector3{-0.976, 2.772, 0.604}, 0.500, 65},
            {Vector3{-1.417, 2.494, 0.877}, 0.500, 66},
            {Vector3{-1.804, 2.121, 1.117}, 0.500, 67},
            {Vector3{-2.121, 1.667, 1.313}, 0.500, 68},
            {Vector3{-2.356, 1.148, 1.459}, 0.500, 69},
            {Vector3{-2.502, 0.585, 1.549}, 0.500, 70},
            {Vector3{-2.551, 0.000, 1.579}, 0.500, 71},
            {Vector3{-0.000, 3.000, 0.000}, 0.500, 72},
            {Vector3{-0.575, 2.942, 0.108}, 0.500, 73},
            {Vector3{-1.129, 2.772, 0.211}, 0.500, 74},
            {Vector3{-1.638, 2.494, 0.306}, 0.500, 75},
            {Vector3{-2.085, 2.121, 0.390}, 0.500, 76},
            {Vector3{-2.452, 1.667, 0.458}, 0.500, 77},
            {Vector3{-2.724, 1.148, 0.509}, 0.500, 78},
            {Vector3{-2.892, 0.585, 0.541}, 0.500, 79},
            {Vector3{-2.949, 0.000, 0.551}, 0.500, 80},
            {Vector3{-0.000, 3.000, -0.000}, 0.500, 81},
            {Vector3{-0.575, 2.942, -0.108}, 0.500, 82},
            {Vector3{-1.129, 2.772, -0.211}, 0.500, 83},
            {Vector3{-1.638, 2.494, -0.306}, 0.500, 84},
            {Vector3{-2.085, 2.121, -0.390}, 0.500, 85},
            {Vector3{-2.452, 1.667, -0.458}, 0.500, 86},
            {Vector3{-2.724, 1.148, -0.509}, 0.500, 87},
            {Vector3{-2.892, 0.585, -0.541}, 0.500, 88},
            {Vector3{-2.949, 0.000, -0.551}, 0.500, 89},
            {Vector3{-0.000, 3.000, -0.000}, 0.500, 90},
            {Vector3{-0.498, 2.942, -0.308}, 0.500, 91},
            {Vector3{-0.976, 2.772, -0.604}, 0.500, 92},
            {Vector3{-1.417, 2.494, -0.877}, 0.500, 93},
            {Vector3{-1.804, 2.121, -1.117}, 0.500, 94},
            {Vector3{-2.121, 1.667, -1.313}, 0.500, 95},
            {Vector3{-2.356, 1.148, -1.459}, 0.500, 96},
            {Vector3{-2.502, 0.585, -1.549}, 0.500, 97},
            {Vector3{-2.551, 0.000, -1.579}, 0.500, 98},
            {Vector3{-0.000, 3.000, -0.000}, 0.500, 99},
            {Vector3{-0.353, 2.942, -0.467}, 0.500, 100},
            {Vector3{-0.692, 2.772, -0.916}, 0.500, 101},
            {Vector3{-1.004, 2.494, -1.330}, 0.500, 102},
            {Vector3{-1.278, 2.121, -1.693}, 0.500, 103},
            {Vector3{-1.503, 1.667, -1.991}, 0.500, 104},
            {Vector3{-1.670, 1.148, -2.212}, 0.500, 105},
            {Vector3{-1.773, 0.585, -2.348}, 0.500, 106},
            {Vector3{-1.808, 0.000, -2.394}, 0.500, 107},
            {Vector3{-0.000, 3.000, -0.000}, 0.500, 108},
            {Vector3{-0.160, 2.942, -0.563}, 0.500, 109},
            {Vector3{-0.314, 2.772, -1.104}, 0.500, 110},
            {Vector3{-0.456, 2.494, -1.603}, 0.500, 111},
            {Vector3{-0.581, 2.121, -2.040}, 0.500, 112},
            {Vector3{-0.683, 1.667, -2.399}, 0.500, 113},
            {Vector3{-0.758, 1.148, -2.666}, 0.500, 114},
            {Vector3{-0.805, 0.585, -2.830}, 0.500, 115},
            {Vector3{-0.821, 0.000, -2.885}, 0.500, 116},
            {Vector3{0.000, 3.000, -0.000}, 0.500, 117},
            {Vector3{0.054, 2.942, -0.583}, 0.500, 118},
            {Vector3{0.106, 2.772, -1.143}, 0.500, 119},
            {Vector3{0.154, 2.494, -1.660}, 0.500, 120},
            {Vector3{0.196, 2.121, -2.112}, 0.500, 121},
            {Vector3{0.230, 1.667, -2.484}, 0.500, 122},
            {Vector3{0.256, 1.148, -2.760}, 0.500, 123},
            {Vector3{0.271, 0.585, -2.930}, 0.500, 124},
            {Vector3{0.277, 0.000, -2.987}, 0.500, 125},
            {Vector3{0.000, 3.000, -0.000}, 0.500, 126},
            {Vector3{0.261, 2.942, -0.524}, 0.500, 127},
            {Vector3{0.512, 2.772, -1.028}, 0.500, 128},
            {Vector3{0.743, 2.494, -1.492}, 0.500, 129},
            {Vector3{0.946, 2.121, -1.899}, 0.500, 130},
            {Vector3{1.112, 1.667, -2.233}, 0.500, 131},
            {Vector3{1.235, 1.148, -2.481}, 0.500, 132},
            {Vector3{1.312, 0.585, -2.634}, 0.500, 133},
            {Vector3{1.337, 0.000, -2.685}, 0.500, 134},
            {Vector3{0.000, 3.000, -0.000}, 0.500, 135},
            {Vector3{0.433, 2.942, -0.394}, 0.500, 136},
            {Vector3{0.848, 2.772, -0.773}, 0.500, 137},
            {Vector3{1.232, 2.494, -1.123}, 0.500, 138},
            {Vector3{1.568, 2.121, -1.429}, 0.500, 139},
            {Vector3{1.843, 1.667, -1.680}, 0.500, 140},
            {Vector3{2.048, 1.148, -1.867}, 0.500, 141},
            {Vector3{2.174, 0.585, -1.982}, 0.500, 142},
            {Vector3{2.217, 0.000, -2.021}, 0.500, 143},
            {Vector3{0.000, 3.000, -0.000}, 0.500, 144},
            {Vector3{0.546, 2.942, -0.211}, 0.500, 145},
            {Vector3{1.071, 2.772, -0.415}, 0.500, 146},
            {Vector3{1.554, 2.494, -0.602}, 0.500, 147},
            {Vector3{1.978, 2.121, -0.766}, 0.500, 148},
            {Vector3{2.326, 1.667, -0.901}, 0.500, 149},
            {Vector3{2.584, 1.148, -1.001}, 0.500, 150},
            {Vector3{2.744, 0.585, -1.063}, 0.500, 151},
            {Vector3{2.797, 0.000, -1.084}, 0.500, 152},
            {Vector3{0.000, 3.000, -0.000}, 0.500, 153},
            {Vector3{0.585, 2.942, -0.000}, 0.500, 154},
            {Vector3{1.148, 2.772, -0.000}, 0.500, 155},
            {Vector3{1.667, 2.494, -0.000}, 0.500, 156},
            {Vector3{2.121, 2.121, -0.000}, 0.500, 157},
            {Vector3{2.494, 1.667, -0.000}, 0.500, 158},
            {Vector3{2.772, 1.148, -0.000}, 0.500, 159},
            {Vector3{2.942, 0.585, -0.000}, 0.500, 160},
            {Vector3{3.000, 0.000, -0.000}, 0.500, 161},
        },
        std::vector<Material>{
            {MaterialType::Diffuse, Vector3{0.329, 0.615, 0.318}},
            {MaterialType::Diffuse, Vector3{0.938, 0.027, 0.806}},
            {MaterialType::Diffuse, Vector3{0.875, 0.326, 0.768}},
            {MaterialType::Mirror, Vector3{0.381, 0.113, 0.864}},
            {MaterialType::Mirror, Vector3{0.361, 0.229, 0.102}},
            {MaterialType::Diffuse, Vector3{0.554, 0.304, 0.438}},
            {MaterialType::Mirror, Vector3{0.898, 0.800, 0.986}},
            {MaterialType::Mirror, Vector3{0.205, 0.074, 0.015}},
            {MaterialType::Mirror, Vector3{0.997, 0.657, 0.784}},
            {MaterialType::Diffuse, Vector3{0.746, 0.459, 0.272}},
            {MaterialType::Diffuse, Vector3{0.391, 0.086, 0.565}},
            {MaterialType::Mirror, Vector3{0.258, 0.679, 0.089}},
            {MaterialType::Diffuse, Vector3{0.290, 0.033, 0.508}},
            {MaterialType::Diffuse, Vector3{0.497, 0.729, 0.094}},
            {MaterialType::Diffuse, Vector3{0.231, 0.624, 0.054}},
            {MaterialType::Diffuse, Vector3{0.994, 0.859, 0.256}},
            {MaterialType::Diffuse, Vector3{0.910, 0.603, 0.929}},
            {MaterialType::Mirror, Vector3{0.629, 0.860, 0.440}},
            {MaterialType::Diffuse, Vector3{0.224, 0.572, 0.142}},
            {MaterialType::Diffuse, Vector3{0.993, 0.327, 0.563}},
            {MaterialType::Diffuse, Vector3{0.551, 0.234, 0.434}},
            {MaterialType::Diffuse, Vector3{0.683, 0.884, 0.085}},
            {MaterialType::Diffuse, Vector3{0.039, 0.423, 0.850}},
            {MaterialType::Diffuse, Vector3{0.518, 0.646, 0.505}},
            {MaterialType::Mirror, Vector3{0.672, 0.678, 0.681}},
            {MaterialType::Diffuse, Vector3{0.663, 0.083, 0.352}},
            {MaterialType::Mirror, Vector3{0.535, 0.191, 0.941}},
            {MaterialType::Diffuse, Vector3{0.663, 0.589, 0.008}},
            {MaterialType::Mirror, Vector3{0.448, 0.006, 0.736}},
            {MaterialType::Diffuse, Vector3{0.949, 0.426, 0.184}},
            {MaterialType::Mirror, Vector3{0.684, 0.736, 0.115}},
            {MaterialType::Diffuse, Vector3{0.651, 0.680, 0.626}},
            {MaterialType::Mirror, Vector3{0.455, 0.518, 0.034}},
            {MaterialType::Mirror, Vector3{0.942, 0.321, 0.810}},
            {MaterialType::Diffuse, Vector3{0.345, 0.637, 0.549}},
            {MaterialType::Mirror, Vector3{0.665, 0.554, 0.029}},
            {MaterialType::Mirror, Vector3{0.814, 0.929, 0.185}},
            {MaterialType::Mirror, Vector3{0.743, 0.795, 0.045}},
            {MaterialType::Diffuse, Vector3{0.266, 0.587, 0.923}},
            {MaterialType::Mirror, Vector3{0.806, 0.530, 0.921}},
            {MaterialType::Mirror, Vector3{0.083, 0.550, 0.314}},
            {MaterialType::Diffuse, Vector3{0.342, 0.395, 0.910}},
            {MaterialType::Diffuse, Vector3{0.241, 0.767, 0.064}},
            {MaterialType::Mirror, Vector3{0.389, 0.529, 0.872}},
            {MaterialType::Mirror, Vector3{0.093, 0.755, 0.445}},
            {MaterialType::Mirror, Vector3{0.061, 0.631, 0.686}},
            {MaterialType::Mirror, Vector3{0.141, 0.661, 0.002}},
            {MaterialType::Diffuse, Vector3{0.770, 0.463, 0.334}},
            {MaterialType::Mirror, Vector3{0.769, 0.135, 0.098}},
            {MaterialType::Mirror, Vector3{0.055, 0.265, 0.891}},
            {MaterialType::Mirror, Vector3{0.197, 0.957, 0.861}},
            {MaterialType::Diffuse, Vector3{0.574, 0.598, 0.729}},
            {MaterialType::Diffuse, Vector3{0.830, 0.700, 0.470}},
            {MaterialType::Mirror, Vector3{0.872, 0.986, 0.371}},
            {MaterialType::Diffuse, Vector3{0.754, 0.747, 0.816}},
            {MaterialType::Mirror, Vector3{0.433, 0.162, 0.434}},
            {MaterialType::Diffuse, Vector3{0.894, 0.611, 0.145}},
            {MaterialType::Mirror, Vector3{0.174, 0.874, 0.618}},
            {MaterialType::Diffuse, Vector3{0.949, 0.595, 0.528}},
            {MaterialType::Mirror, Vector3{0.552, 0.959, 0.510}},
            {MaterialType::Diffuse, Vector3{0.038, 0.006, 0.942}},
            {MaterialType::Diffuse, Vector3{0.596, 0.102, 0.818}},
            {MaterialType::Diffuse, Vector3{0.477, 0.506, 0.701}},
            {MaterialType::Mirror, Vector3{0.486, 0.299, 0.026}},
            {MaterialType::Mirror, Vector3{0.513, 0.590, 0.117}},
            {MaterialType::Mirror, Vector3{0.449, 0.913, 0.882}},
            {MaterialType::Diffuse, Vector3{0.959, 0.858, 0.437}},
            {MaterialType::Diffuse, Vector3{0.965, 0.434, 0.665}},
            {MaterialType::Diffuse, Vector3{0.827, 0.569, 0.715}},
            {MaterialType::Diffuse, Vector3{0.008, 0.608, 0.053}},
            {MaterialType::Mirror, Vector3{0.430, 0.589, 0.318}},
            {MaterialType::Mirror, Vector3{0.333, 0.094, 0.032}},
            {MaterialType::Mirror, Vector3{0.317, 0.303, 0.683}},
            {MaterialType::Diffuse, Vector3{0.005, 0.041, 0.381}},
            {MaterialType::Mirror, Vector3{0.412, 0.909, 0.411}},
            {MaterialType::Mirror, Vector3{0.852, 0.061, 0.509}},
            {MaterialType::Diffuse, Vector3{0.318, 0.558, 0.898}},
            {MaterialType::Mirror, Vector3{0.022, 0.199, 0.715}},
            {MaterialType::Diffuse, Vector3{0.841, 0.756, 0.563}},
            {MaterialType::Diffuse, Vector3{0.413, 0.196, 0.282}},
            {MaterialType::Mirror, Vector3{0.339, 0.674, 0.634}},
            {MaterialType::Mirror, Vector3{0.017, 0.125, 0.719}},
            {MaterialType::Diffuse, Vector3{0.366, 0.450, 0.917}},
            {MaterialType::Mirror, Vector3{0.807, 0.124, 0.792}},
            {MaterialType::Diffuse, Vector3{0.351, 0.632, 0.046}},
            {MaterialType::Mirror, Vector3{0.623, 0.087, 0.294}},
            {MaterialType::Diffuse, Vector3{0.048, 0.453, 0.515}},
            {MaterialType::Mirror, Vector3{0.018, 0.235, 0.918}},
            {MaterialType::Diffuse, Vector3{0.477, 0.395, 0.238}},
            {MaterialType::Diffuse, Vector3{0.168, 0.915, 0.565}},
            {MaterialType::Mirror, Vector3{0.568, 0.784, 0.349}},
            {MaterialType::Diffuse, Vector3{0.554, 0.750, 0.364}},
            {MaterialType::Mirror, Vector3{0.643, 0.474, 0.479}},
            {MaterialType::Diffuse, Vector3{0.381, 0.405, 0.162}},
            {MaterialType::Mirror, Vector3{0.455, 0.972, 0.131}},
            {MaterialType::Mirror, Vector3{0.350, 0.237, 0.088}},
            {MaterialType::Mirror, Vector3{0.949, 0.566, 0.367}},
            {MaterialType::Diffuse, Vector3{0.758, 0.647, 0.628}},
            {MaterialType::Mirror, Vector3{0.654, 0.410, 0.637}},
            {MaterialType::Diffuse, Vector3{0.518, 0.860, 0.535}},
            {MaterialType::Diffuse, Vector3{0.902, 0.696, 0.943}},
            {MaterialType::Mirror, Vector3{0.310, 0.961, 0.250}},
            {MaterialType::Mirror, Vector3{0.744, 0.653, 0.304}},
            {MaterialType::Mirror, Vector3{0.290, 0.201, 0.254}},
            {MaterialType::Mirror, Vector3{0.512, 0.679, 0.938}},
            {MaterialType::Diffuse, Vector3{0.032, 0.769, 0.258}},
            {MaterialType::Mirror, Vector3{0.096, 0.524, 0.640}},
            {MaterialType::Mirror, Vector3{0.311, 0.473, 0.033}},
            {MaterialType::Mirror, Vector3{0.845, 0.121, 0.952}},
            {MaterialType::Diffuse, Vector3{0.413, 0.714, 0.126}},
            {MaterialType::Mirror, Vector3{0.921, 0.986, 0.644}},
            {MaterialType::Diffuse, Vector3{0.709, 0.495, 0.501}},
            {MaterialType::Diffuse, Vector3{0.794, 0.867, 0.483}},
            {MaterialType::Mirror, Vector3{0.007, 0.957, 0.351}},
            {MaterialType::Mirror, Vector3{0.002, 0.728, 0.085}},
            {MaterialType::Mirror, Vector3{0.893, 0.733, 0.410}},
            {MaterialType::Diffuse, Vector3{0.663, 0.573, 0.380}},
            {MaterialType::Mirror, Vector3{0.059, 0.339, 0.258}},
            {MaterialType::Mirror, Vector3{0.029, 0.859, 0.595}},
            {MaterialType::Mirror, Vector3{0.952, 0.053, 0.194}},
            {MaterialType::Mirror, Vector3{0.975, 0.839, 0.794}},
            {MaterialType::Mirror, Vector3{0.216, 0.690, 0.124}},
            {MaterialType::Diffuse, Vector3{0.578, 0.718, 0.240}},
            {MaterialType::Mirror, Vector3{0.430, 0.469, 0.692}},
            {MaterialType::Mirror, Vector3{0.391, 0.007, 0.295}},
            {MaterialType::Diffuse, Vector3{0.368, 0.157, 0.964}},
            {MaterialType::Mirror, Vector3{0.408, 0.310, 0.509}},
            {MaterialType::Diffuse, Vector3{0.571, 0.976, 0.677}},
            {MaterialType::Mirror, Vector3{0.246, 0.379, 0.546}},
            {MaterialType::Mirror, Vector3{0.044, 0.591, 0.391}},
            {MaterialType::Mirror, Vector3{0.559, 0.776, 0.961}},
            {MaterialType::Mirror, Vector3{0.788, 0.702, 0.897}},
            {MaterialType::Mirror, Vector3{0.082, 0.351, 0.838}},
            {MaterialType::Mirror, Vector3{0.082, 0.280, 0.557}},
            {MaterialType::Mirror, Vector3{0.476, 0.051, 0.819}},
            {MaterialType::Diffuse, Vector3{0.989, 0.466, 0.896}},
            {MaterialType::Mirror, Vector3{0.041, 0.284, 0.306}},
            {MaterialType::Mirror, Vector3{0.326, 0.021, 0.206}},
            {MaterialType::Diffuse, Vector3{0.242, 0.621, 0.612}},
            {MaterialType::Mirror, Vector3{0.838, 0.806, 0.292}},
            {MaterialType::Diffuse, Vector3{0.497, 0.817, 0.849}},
            {MaterialType::Mirror, Vector3{0.863, 0.078, 0.482}},
            {MaterialType::Mirror, Vector3{0.837, 0.786, 0.561}},
            {MaterialType::Mirror, Vector3{0.582, 0.916, 0.055}},
            {MaterialType::Mirror, Vector3{0.001, 0.083, 0.412}},
            {MaterialType::Mirror, Vector3{0.741, 0.010, 0.934}},
            {MaterialType::Diffuse, Vector3{0.159, 0.684, 0.917}},
            {MaterialType::Diffuse, Vector3{0.779, 0.457, 0.938}},
            {MaterialType::Diffuse, Vector3{0.510, 0.779, 0.784}},
            {MaterialType::Diffuse, Vector3{0.849, 0.641, 0.899}},
            {MaterialType::Mirror, Vector3{0.603, 0.549, 0.731}},
            {MaterialType::Diffuse, Vector3{0.634, 0.527, 0.844}},
            {MaterialType::Diffuse, Vector3{0.479, 0.232, 0.347}},
            {MaterialType::Diffuse, Vector3{0.270, 0.631, 0.106}},
            {MaterialType::Diffuse, Vector3{0.652, 0.077, 0.468}},
            {MaterialType::Mirror, Vector3{0.118, 0.759, 0.415}},
            {MaterialType::Diffuse, Vector3{0.878, 0.732, 0.718}},
            {MaterialType::Diffuse, Vector3{0.342, 0.479, 0.721}},
            {MaterialType::Diffuse, Vector3{0.106, 0.974, 0.771}},
            {MaterialType::Diffuse, Vector3{0.355, 0.022, 0.144}},
            {MaterialType::Mirror, Vector3{0.612, 0.975, 0.641}},
            {MaterialType::Mirror, Vector3{0.879, 0.247, 0.050}},
        }, 
        std::vector<PointLight>{
            // a strong point light "right under" origin
            {Vector3{100, 100, 100}, Vector3{0, -3, 0}}
        }
    };

    // duplicate scene_1 except for MaterialType
    Scene hw1_scene_6{
        Camera{
            Vector3{0, 0, 0},  // lookfrom
            Vector3{0, 0, -1}, // lookat
            Vector3{0, 1, 0},  // up
            45                 // vfov
        },
        std::vector<Sphere>{
            {Vector3{0.0, -100.5, -3.0}, 100.0, 0},
            {Vector3{0.0, 0.0, -3.0}, 0.5, 1},
            {Vector3{1.0, 0.0, -3.0}, 0.5, 2},
            {Vector3{-1.0, 0.0, -3.0}, 0.5, 3}},
        std::vector<Material>{
            {MaterialType::Diffuse, Vector3{0.80, 0.80, 0.20}},
            {MaterialType::Mirror, Vector3{0.75, 0.25, 0.25}},
            {MaterialType::Glass, Vector3{0.75, 0.25, 0.75}},
            {MaterialType::Diamond, Vector3{0.25, 0.75, 0.75}},
        },
        std::vector<PointLight>{
            {Vector3{100, 100, 100}, Vector3{5, 5, 2}},
            {Vector3{10, 10, 10}, Vector3{-5, 5, 1}},
            {Vector3{2, 2, 2}, Vector3{0, 5, -5}}}};

    Scene hw1_scenes[] = {
        hw1_scene_0, hw1_scene_1, hw1_scene_2, hw1_scene_3, hw1_scene_4,
        hw1_scene_5, // my own scene
        hw1_scene_6     // with refractor material
    };

}
