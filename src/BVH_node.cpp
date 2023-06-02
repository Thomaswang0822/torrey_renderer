#include "BVH_node.h"
#include "helper.h"

using namespace std;

BVH_node::BVH_node(shared_ptr<Shape> obj) {
    leafObj = obj;
    box = get_bbox(obj);
}


BVH_node::BVH_node(std::vector<shared_ptr<Shape>>& objects, Scene& scene,
                pcg32_state &rng) {
    // traverse the entire object by index, build BVH_node 
    // for single sphere or TriangleMesh
    std::vector<std::shared_ptr<BVH_node>> meshBVH;
    size_t idx = 0;
    Shape* shapePtr;
    while (idx < objects.size()) {
        shapePtr = objects[idx].get();
        if (std::get_if<Sphere>(shapePtr)) {
            // call leaf constructor on single sphere
            meshBVH.push_back(std::make_shared<BVH_node>(objects[idx]));
            idx++;
        }
        else if (Triangle* tri = std::get_if<Triangle>(shapePtr)) {
            // get its mesh count
            int meshSize = 1;
            if (tri->mesh_id != -1) {
                meshSize = scene.meshes[tri->mesh_id].indices.size();
            }
            // call ordinary constructor
            meshBVH.push_back(std::make_shared<BVH_node>(
                objects,
                idx, idx + meshSize,
                rng, false
            ));
            // make jump
            idx += meshSize;
        }
    }

    // rare case: single mesh/Sphere
    if (meshBVH.size() == 1) {
        left = meshBVH[0].get()->left;
        right = meshBVH[0].get()->right;
        leafObj = meshBVH[0].get()->leafObj;
        box = AABB(meshBVH[0].get()->box);
        return;
    }

    // sort these BVH_node using their bbox center
    Vector3 camOrigin = scene.camera.origin;
    std::sort(meshBVH.begin(), meshBVH.end(),
              [&camOrigin](const std::shared_ptr<BVH_node> a, const std::shared_ptr<BVH_node> b) {
                  double dist_a = distance_squared(a.get()->box.center(), camOrigin);
                  double dist_b = distance_squared(b.get()->box.center(), camOrigin);
                  return dist_a < dist_b;
              }  // lambda function
    );

    // merge these BVH_node together an form a Single BVH_node
    int head = 0; int tail = meshBVH.size()-1;
    std::shared_ptr<BVH_node> leftPtr = meshBVH[head];
    std::shared_ptr<BVH_node> rightPtr = meshBVH[tail];
    // left and right at the same time to keep the tree more balanced
    while (tail - head >= 3) {  // when both can merge
        leftPtr = std::make_shared<BVH_node>(leftPtr, meshBVH[++head]);
        rightPtr = std::make_shared<BVH_node>(meshBVH[--tail], rightPtr);    
    }
    if (tail-head == 2) {
        leftPtr = std::make_shared<BVH_node>(leftPtr, meshBVH[++head]);
    }
    // Now left and right cover all nodes, construct self
    left = leftPtr;
    right = rightPtr;
    box = surrounding_box(leftPtr.get()->box, rightPtr.get()->box);
}


BVH_node::BVH_node(std::vector<std::shared_ptr<Shape>>& objects,
        size_t start, size_t end, pcg32_state &rng, bool randomAxis)
{
    // Timer timer;
    // SHOULD NOT: Create a modifiable array of the source scene objects
    // auto objects = src_objects;

    int axis;
    Vector3 rangeXYZ;
    if (randomAxis) {
        axis = random_int(0,2, rng);
    } else {
        // Quiz: try picking axis with largest range
        rangeXYZ = axisRange(objects, start, end);
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
        // tick(timer);
        auto mid = SAH_split(objects, start, end, axis);
        // auto mid = start + object_span/2;
        left = make_shared<BVH_node>(objects, start, mid, rng, randomAxis);
        right = make_shared<BVH_node>(objects, mid, end, rng, randomAxis);
        /* if (object_span >= 10000){
            std::cout << "Recursion left and right from " << start << " to " << end << 
                "\n\t took another " << tick(timer) << " seconds." << std::endl;
        } */
    }
    box = surrounding_box(left.get()->box, right.get()->box);
}


Vector3 axisRange(const vector<shared_ptr<Shape>>& src_objects, size_t start, size_t end) {
    Vector3 resMin(infinity<Real>(), infinity<Real>(), infinity<Real>());
    Vector3 resMax(-resMin);

    Shape* shapePtr = nullptr;
    Vector3 currMin, currMax;
    for (size_t i=start; i<end; ++i) {
        shapePtr = src_objects[i].get();
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
        resMin = min(resMin, currMin);
        resMax = max(resMax, currMax);
    }
    return resMax - resMin;
}


bool BVH_node::hit(const ray& r, Real t_min, Real t_max,
            const Scene& scene, Hit_Record& rec, Shape*& hitObj)
{
    // complete miss
    if (!box.hit(r, t_min, t_max)) {
        return false;
    }

    // std::cout << "Not always miss top-level box \n";
    // leaf node
    if (leafObj) {
        checkRayShapeHit(r, *leafObj, rec, hitObj);
        // return leafObj.get() == hitObj;
        return bool(t_min <= rec.dist && rec.dist <= t_max);
    }

    // non-leaf node: recursion
    bool hit_left, hit_right;
    if ( leftBoxCloser(r.orig) ) {
        hit_left = left->hit(r, t_min, t_max, scene, rec, hitObj);
        hit_right = right->hit(r, t_min, 
            hit_left ? rec.dist : t_max,  // spatial short-circuiting
            scene, rec, hitObj
        );
    } else {
        hit_right = right->hit(r, t_min, t_max, scene, rec, hitObj);
        hit_left = left->hit(r, t_min, 
            hit_right ? rec.dist : t_max,  // spatial short-circuiting
            scene, rec, hitObj
        );
    }
    

    return hit_left || hit_right;
}

size_t SAH_split(std::vector<std::shared_ptr<Shape>>& objects,
        size_t start, size_t end, int axis) 
{
    // Step 1: determine how many different divisions to use
    size_t nDiv = std::min(size_t(1024), (end-start));
    // std::cout << "Step 1 Done: " << std::endl;

    // Step 2: create bbox for each division
    std::vector<AABB> divBoxes;
    size_t jump = (end-start) / nDiv;
    size_t localStart = start;
    for (size_t i=0; i<nDiv-1; ++i) {
        divBoxes.push_back(rangeAABB(objects, localStart, localStart+jump));
        localStart += jump;
    }
    // deal with the last entry, which have a larger jump
    divBoxes.push_back(rangeAABB(objects, localStart, end));
    assert(divBoxes.size() == nDiv && "divBoxes caculation is wrong");
    // std::cout << "Step 2 Done: " << std::endl;

    // Step 3: calculate SAH at each division choice
    // *Simplyfy: we don't need the following parts in formal SAH to make decision
    //   cost of traversal current node
    //   surface area of outside AABB (of current node)
    //   cost of each primitive, since we assume they (for Tri and Sphere) be the same.
    // *We use accumulation trick to merge from left to right once and right to left once.
    std::vector<Real> choices;
    choices.resize(nDiv - 1);   // 16 divisions give you 15 choices of split position
    // sweep forward to get SAH of left in linear time
    AABB currAABB = divBoxes[0];
    choices[0] = currAABB.surfaceA() * jump * (0+1);
    for (size_t i=1; i<nDiv-1; ++i) {  // don't include last bbox (full box)
        // merge
        currAABB = surrounding_box(currAABB, divBoxes[i]);
        // calculate left SAH
        choices[i] = currAABB.surfaceA() * jump * (i+1);
    }
    // std::cout << "Sweep forward Done: " << std::endl;
    // sweep backward; this time index is off by 1
    currAABB = divBoxes[nDiv-1];
    choices[nDiv-2] += currAABB.surfaceA() * (end-localStart);
    for (size_t i=nDiv-2; i>0; i--) {  // don't include first bbox (full box)
        // merge
        // std::cout << nDiv << '\t' << i+1u << std::endl;
        currAABB = surrounding_box(currAABB, divBoxes[i]);
        // calculate right SAH; += 
        choices[i-1] += currAABB.surfaceA() * (jump * (nDiv-1-i) + (end-localStart));
    }
    // now choices store the following
    // choices[i] = leftSA * #primLeft + rightSA * #primRight
    // std::cout << "Step 3 Done: " << std::endl;


    // Step 4: get the choice index of smallest SAH
    auto smallest_it = std::min_element(choices.begin(), choices.end());
    int smallest_index = std::distance(choices.begin(), smallest_it);

    // Step 5: convert choice index into object index
    // index == 0 means left has 1 chunk
    return start + jump * (smallest_index + 1);
}


AABB rangeAABB(std::vector<std::shared_ptr<Shape>>& objects,
        size_t start, size_t end) 
{
    if (end == start + 1) {
        return get_bbox(objects[start]);
    }

    AABB currBox = get_bbox(objects[start]);
    Vector3 localMin = Vector3(currBox.minimum);
    Vector3 localMax = Vector3(currBox.maximum);

    for (auto i=start; i<end; ++i){
        currBox = get_bbox(objects[i]);
        localMin = min(localMin, currBox.minimum);
        localMax = max(localMax, currBox.maximum);
    }

    return AABB(localMin, localMax);
}


/* ### BVH-version ### */
bool isVisible(const Vector3& shadingPt, Vector3& lightPos, Scene& scene, BVH_node& root) {
    double d = distance(shadingPt, lightPos);
    // shot ray from light to shadingPt
    ray lightRay(lightPos, shadingPt, true);
    // test hitting point with BVH
    Hit_Record rec;
    Shape* hitObj = nullptr;
    // hit => not visible (shadow)
    return !root.hit(lightRay, EPSILON, (1-EPSILON) * d, scene, rec, hitObj);
}
