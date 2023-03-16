#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <vector>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

int axisToSplit(Vector3D input) {
    int index = 0;
    double curMax = 0;
    for (int i = 0; i < 2; ++i) {
        if (input[i] >= curMax) {
            curMax = input[i];
            index = i;
        }
    }
    return index;
}


BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {
    
    // TODO (Part 2.1):
    // Construct a BVH from the given vector of primitives and maximum leaf
    // size configuration. The starter code build a BVH aggregate with a
    // single leaf node (which is also the root) that encloses all the
    // primitives.
//
//    BBox bbox;
//
//      for (auto p = start; p != end; p++) {
//        BBox bb = (*p)->get_bbox();
//        bbox.expand(bb);
//      }
//
//      BVHNode *node = new BVHNode(bbox);
//      node->start = start;
//      node->end = end;
//
//      return node;

    size_t primitive_count = 0;
    std::vector<double> minLayer;
    std::vector<double> maxLayer;

    BBox bbox;
    for (auto p = start; p != end; p++) {
        primitive_count++; // increment primitive count
        BBox bb = (*p)->get_bbox(); // add to current bounding box
        Vector3D centroid = bb.centroid();
        bbox.expand(bb);
    }
    // base case
    if (primitive_count <= max_leaf_size) {
        BVHNode *node = new BVHNode(bbox);
        node->start = start;
        node->end = end;
        return node;
    }
    // splitting along axis with greatest max-min difference
    int axis = axisToSplit(bbox.extent);
    double splitPoint = 0;
    for (auto p = start; p != end; p++) {
        splitPoint += (*p)->get_bbox().centroid()[axis];
    }
    splitPoint = splitPoint / primitive_count;
    std::vector<Primitive *> *left = new std::vector<Primitive *>;
    std::vector<Primitive *> *right = new std::vector<Primitive *>;
    for (auto p = start; p != end; p++) {
        Vector3D centroid = (*p)->get_bbox().centroid();
        if (centroid[axis] <= splitPoint) {
            left->push_back(*p);
        } else {
            right->push_back(*p);
        }
    }
    // checking if splitting resulted in either array being empty
    if (left->size() == 0) {
        int rm = 0;
        for (auto i = right->size() - 1; i > right->size() / 2; i--) {
            left->push_back((*right)[i]);
            rm++;
        }
        for (int _ = 0; _ < rm; _++) {
            right->pop_back();
        }
    }
    if (right->size() == 0) {
        int rm = 0;
        for (auto i = left->size() - 1; i > left->size() / 2; i--) {
            right->push_back((*left)[i]);
            rm++;
        }
        for (int _ = 0; _ < rm; _++) {
            left->pop_back();
        }
    }
    BVHNode *node = new BVHNode(bbox);
    BVHNode *leftNode = construct_bvh(left->begin(), left->end(), max_leaf_size);
    BVHNode *rightNode = construct_bvh(right->begin(), right->end(), max_leaf_size);
    node->l = leftNode;
    node->r = rightNode;
    return node;
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.
//    for (auto p : primitives) {
//      total_isects++;
//      if (p->has_intersection(ray))
//        return true;
//    }
//    return false;

    return intersect(ray, nullptr);
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
    // TODO (Part 2.3):
    // Fill in the intersect function.
    
    
//    bool hit = false;
//    for (auto p : primitives) {
//      total_isects++;
//      hit = p->intersect(ray, i) || hit;
//    }
//    return hit;

    if (node == nullptr) {
        return false;
    }
    if (!(node->bb.intersect(ray, ray.min_t, ray.max_t))) {
        return false;
    }
    bool didHit = false;
    if (node->isLeaf()) {
        for (auto p = node->start; p != node->end; p++) {
            total_isects++;
            if ((*p)->intersect(ray, i)) {
                didHit = true;
            }
        }
    }
    
    bool hitLeft = intersect(ray, i, node->l);
    bool hitRight = intersect(ray, i, node->r);
    return didHit || hitLeft || hitRight;
}
} // namespace SceneObjects
} // namespace CGL
