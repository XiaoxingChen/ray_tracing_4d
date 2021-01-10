#if !defined(_BOUNDING_VOLUME_HIERARCHY_H_)
#define _BOUNDING_VOLUME_HIERARCHY_H_

#include "axis_aligned_bounding_box.h"
#include <vector>
#include <memory>
#include "hittable.h"


namespace rtc
{
namespace bvh
{
class Node;
using NodePtr = std::unique_ptr<Node>;
class Node
{
public:
    Node(size_t dim, bool is_leaf=false):aabb_(dim), is_leaf_(is_leaf) {}
    void addChild(NodePtr p_node) {}

    std::vector<NodePtr>& children() { return children_; }
    std::vector<Hittable>& hittables() { return hittables_; }
    const AABB& boundingBox() const { return aabb_; }

    Hittable::HitRecordPtr hit(Ray& ray)
    {
        if(!aabb_.hit(ray)) return nullptr;
        return is_leaf_ ? hitLeaf(ray) : hitInternalNode(ray);
    }

private:
    Hittable::HitRecordPtr hitLeaf(Ray& ray)
    {
        Hittable::HitRecordPtr result = nullptr;
        for(auto & hittable : hittables_)
        {
            auto p_record = hittable.rigidBody().hit(ray);
            if(!p_record) continue;

            result = std::make_shared<Hittable::HitRecord>(
                hittable.material().attenuation(),
                hittable.material().scatter(ray, p_record->p, p_record->n),
                p_record->t);

            ray.tMax() = p_record->t;
        }
        return result;
    }

    Hittable::HitRecordPtr hitInternalNode(Ray& ray)
    {
        Hittable::HitRecordPtr result(nullptr);
        for(auto & child: children_)
        {
            auto p_record = child->hit(ray);
            if(!p_record) continue;
            result = p_record;
            ray.tMax() = p_record->hit_t;
        }
        return result;
    }

private:
    AABB aabb_;
    std::vector<NodePtr> children_;
    bool is_leaf_;
    std::vector<Hittable> hittables_;
};

inline NodePtr build(
    NodePtr root,
    // const std::vector<Hittable>& hittables,
    size_t max_hittable_per_leaf=4)
{
    if(root->children().size() <= max_hittable_per_leaf)
        return std::move(root);

    size_t longest_axis = root->boundingBox().axesByLength().back();
    FloatType center = root->boundingBox().center()(longest_axis);
    //TODO

}

} // namespace bvh
} // namespace rtc




#endif // _BOUNDING_VOLUME_HIERARCHY_H_
