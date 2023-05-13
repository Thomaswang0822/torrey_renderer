#pragma once

#include "Scene.h"

#include <memory>

using namespace std;

struct BVH_node {
    // member variables
    shared_ptr<BVH_node> left, right;
    AABB box;
    shared_ptr<Shape> leafObj;  // nullptr if non-leaf node

    // leaf constructor
    BVH_node(shared_ptr<Shape> obj);
    // quick merge constructor
    BVH_node(shared_ptr<BVH_node> leftBVH, shared_ptr<BVH_node> rightBVH) :
    left(leftBVH), right(rightBVH)
    {
        box = surrounding_box(left.get()->box, right.get()->box);
    }
    // wrapper around constructor below: force BVH for each mesh
    BVH_node(vector<shared_ptr<Shape>>& objects, Scene& scene, pcg32_state &rng);
    // top-level recursive constructor
    BVH_node(vector<shared_ptr<Shape>>& objects,
        size_t start, size_t end,  // index
        pcg32_state &rng, bool randomAxis=true  // to pick random axis
    );
    /***DEBUG NOTE
     * Instead of a vector<Shape>&, we need to pass in a vector<shared_ptr<Shape>>&
     * This is because a reference to a vector itself doesn't give reference to
     * element (our Shape).
     * Thus, we need to use a vector of pointers.
     */

    // member functions
    bool hit(const ray& r, Real t_min, Real t_max,
            const Scene& scene, Hit_Record& rec, Shape*& hitObj);
    
    bool write_bounding_box(AABB& output_box) const {
        output_box = box;
        return true;
    }

    bool leftBoxCloser(Vector3 origin) {
        return distance_squared(left.get()->box.center(), origin) < 
                distance_squared(right.get()->box.center(), origin);
    }
    
};

inline int random_int(int min, int max, pcg32_state &rng) {
    // Returns a random integer in [min,max].
    return static_cast<int>(
        next_pcg32_real<double>(rng) * (max-min+1)
    ) + min;
}

// Comparator that works on bbox of primitives (Shape)
inline bool box_compare(const shared_ptr<Shape> a, const shared_ptr<Shape> b, int axis) {
    // assert(0 <= axis && axis <= 2);

    return get_bbox(a).minimum[axis] < get_bbox(b).minimum[axis];
}

inline bool box_x_compare (const shared_ptr<Shape> a, const shared_ptr<Shape> b) {
    return box_compare(a, b, 0);
}

inline bool box_y_compare (const shared_ptr<Shape> a, const shared_ptr<Shape> b) {
    return box_compare(a, b, 1);
}

inline bool box_z_compare (const shared_ptr<Shape> a, const shared_ptr<Shape> b) {
    return box_compare(a, b, 2);
}

// Helper for picking axis with largest range
Vector3 axisRange(const vector<shared_ptr<Shape>>& src_objects, size_t start, size_t end);

inline int maxRangeIndex(Vector3 rangeXYZ) {
    int max_index = 0;
    Real max_value = rangeXYZ[0];
    for (int i = 1; i < 3; i++) {
        if (rangeXYZ[i] > max_value) {
            max_value = rangeXYZ[i];
            max_index = i;
        }
    }
    return max_index;
}

/**
 * @brief Given related information, determine the split index by SAH
 * 
 * @param objects 
 * @param start 
 * @param end 
 * @param axis: which axis to use.
 * @return size_t the optimal index (start <= mid < end) of split
 */
size_t SAH_split(std::vector<std::shared_ptr<Shape>>& objects,
        size_t start, size_t end, int axis);

// We create this function because it's potentially
// faster than merging AABB one by one
AABB rangeAABB(std::vector<std::shared_ptr<Shape>>& objects,
        size_t start, size_t end);

/* ### BVH-version ### */

// BVH_RaySceneHit() is actually BVH_node::hit
bool BVH_isVisible(Vector3& shadingPt, Vector3& lightPos, Scene& scene, BVH_node& root);
Vector3 BVH_DiffuseColor(Scene& scene, Hit_Record& rec, const Color& refl, 
                        BVH_node& root, const Shape* hitObj, pcg32_state& rng);
Vector3 BVH_PixelColor(Scene& scene, ray& localRay, BVH_node& root, 
                        pcg32_state& rng, unsigned int recDepth=MAX_DEPTH);

/**
 * @brief Compute the total SCALED contribution from an area-light TriangleMesh.
 * Sampling from mesh (alternative to looking at all Triangles) has been implemented.
 * It will be activated if sampleAll == true && mesh size >= maxSample
 * 
 * @note Each time we sample a point x_i from Triangle_i, the p(x) is just 1/area_i.
 *   Here we "expect" small contribution from each Triangle sum up to the total contribution
 *   of the mesh, so we don't take the average.
 * @note If we sample from mesh first, the probability of a Triangle_i being picked is
 *   its normalized area (area_i/mesh_surface_area). So overall, the p(x) = Prob(pick Triangle_i 
 *   from mesh) * Prob(pick x_i from Triangle_i) = area_i/mesh_surface_area * 1/area_i = 
 *   1/mesh_surface_area. In this sense, we "expect" each single sampling point to represent
 *   the total contribution from the mesh. Thus, we take the average at last. Quite counter-intuitive.
 * @note number of samples we really take is min(mesh.size(), maxSample). i.e. if the mesh only has
 *   16 Triangles, we take 16 samples instead of 64.
 * 
 * @param Kd diffuse surface reflectance at the hitting point
 * @param I area light radiance
 * @param sampleAll user defined flag of whether to sample lights
 *   (instead of looking at all lights in the mesh)
 * @param maxSample max number of light samples we pick from the mesh, default to 64
 * @param stratified whether use strafitied sampling. See helper::Triangle_sample()
 * 
 * @return Vector3 
 */
Vector3 meshLight_total_contribution(Scene& scene, Hit_Record& rec, BVH_node& root,
            int mesh_id, int shape_id,
            const Vector3& Kd, const Vector3& I,
            pcg32_state& rng, 
            bool sampleAll=true, int maxSample=64,
            bool stratified=false);



Vector3 sphereLight_contribution(Scene& scene, Hit_Record& rec, BVH_node& root,
            const Shape* lightObj, 
            const Vector3& Kd, const Vector3& I,
            pcg32_state& rng, 
            int ct=1);