#pragma once

#include "Scene.h"
#include "ray.h"
#include "hw2.h"

#include <memory>

using namespace std;

struct BVH_node {
    // member variables
    shared_ptr<BVH_node> left, right;
    AABB box;
    shared_ptr<Shape> leafObj;  // nullptr if non-leaf node

    // leaf constructor
    BVH_node(shared_ptr<Shape> obj);
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
            const Scene& scene, Real& hitDist, Shape*& hitObj);
    
    bool write_bounding_box(AABB& output_box) const {
        output_box = box;
        return true;
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