#include "BVH_node.h"

#include <algorithm>


BVH_node::BVH_node(Shape& obj) {
    leafObj = &obj;
    box = get_bbox(obj);
}


BVH_node::BVH_node(const vector<Shape>& src_objects,
        size_t start, size_t end, pcg32_state &rng, bool randomAxis)
{
    // Create a modifiable array of the source scene objects
    auto objects = src_objects;

    int axis;
    Vector3 rangeXYZ;
    if (randomAxis) {
        axis = random_int(0,2, rng);
    } else {
        // Quiz: try picking axis with largest range
        rangeXYZ = axisRange(src_objects, start, end);
        axis = maxRangeIndex(rangeXYZ);
    }

    auto comparator = (axis == 0) ? box_x_compare
                    : (axis == 1) ? box_y_compare
                                  : box_z_compare;

    size_t object_span = end - start;

    if (object_span == 1) {
        left = right = make_shared<BVH_node>(objects[start]);
        // std::cout << left.get()->leafObj << "\t" << 
        //     bool(left.get()->leafObj == nullptr) << std::endl;
    } else if (object_span == 2) {
        if (comparator(objects[start], objects[start+1])) {
            left = make_shared<BVH_node>(objects[start]);
            right = make_shared<BVH_node>(objects[start+1]);
        } else {
            left = make_shared<BVH_node>(objects[start+1]);
            right = make_shared<BVH_node>(objects[start]);
        }
    } else {
        std::sort(objects.begin() + start, objects.begin() + end, comparator);

        auto mid = start + object_span/2;
        left = make_shared<BVH_node>(objects, start, mid, rng, randomAxis);
        right = make_shared<BVH_node>(objects, mid, end, rng, randomAxis);
    }

    AABB box_left, box_right;

    if (  !left->write_bounding_box (box_left)
       || !right->write_bounding_box(box_right)
    )
        std::cerr << "No bounding box in bvh_node constructor.\n";

    box = surrounding_box(box_left, box_right);
}


Vector3 axisRange(const vector<Shape>& src_objects, size_t start, size_t end) {
    Vector3 resMin(infinity<Real>(), infinity<Real>(), infinity<Real>());
    Vector3 resMax(-resMin);

    Shape* shapePtr = nullptr;
    Vector3 currMin, currMax;
    for (size_t i=start; i<end; ++i) {
        if (Sphere *sph = std::get_if<Sphere>(shapePtr)) {
            // Vector3 - Real has been overloaded
            currMin = sph->box.minimum;
            currMax = sph->box.maximum;
        } else if (Triangle *tri = std::get_if<Triangle>(shapePtr)) {
            currMin = tri->box.minimum;
            currMax = tri->box.maximum;
        } else {
            assert(false);
        }
        // update result
        resMin = {
            std::min(resMin.x, currMin.x),
            std::min(resMin.y, currMin.y),
            std::min(resMin.z, currMin.z)
        };
        resMax = {
            std::max(resMax.x, currMax.x),
            std::max(resMax.y, currMax.y),
            std::max(resMax.z, currMax.z)
        };
    }
    return resMax - resMin;
}


bool BVH_node::hit(const ray& r, Real t_min, Real t_max,
            const Scene& scene, Real& hitDist, Shape*& hitObj)
{
    // complete miss
    if (!box.hit(r, t_min, t_max)) {
        return false;
    }

    // std::cout << "Not always miss top-level box \n";
    // leaf node
    if (leafObj) {
        // std::cout << "checkRayShapeHit called \n";
        checkRayShapeHit(r, *leafObj, hitDist, hitObj);
        // std::cout << hitDist << std::endl;
        // return or false? Return value here shouldn't matter
        return leafObj == hitObj;
    }

    // non-leaf node: recursion
    bool hit_left = left->hit(r, t_min, t_max, scene, hitDist, hitObj);
    bool hit_right = right->hit(r, t_min, 
        hit_left ? hitDist : t_max,  // spatial short-circuiting
        scene, hitDist, hitObj
    );

    return hit_left || hit_right;
}
