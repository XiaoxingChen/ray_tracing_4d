#if !defined(_BOUNDING_VOLUME_HIERARCHY_H_)
#define _BOUNDING_VOLUME_HIERARCHY_H_

#include "axis_aligned_bounding_box.h"
#include <vector>
#include <memory>
#include "hittable.h"
#include <algorithm>


namespace rtc
{
namespace bvh
{
class Node;
size_t sortInLongestAxis(
    HittableBufferPtr buffer,
    const std::array<size_t, 2>& range,
    size_t dim,
    bool verbose);
using NodePtr = std::unique_ptr<Node>;
class Node
{
public:

    Node(size_t dim,
        HittableBufferPtr hittable_buffer,
        const std::array<size_t, 2>& hittable_range,
        bool is_leaf=true)
        :aabb_(dim), is_leaf_(is_leaf),
        hittable_buffer_(hittable_buffer),
        hittable_range_(hittable_range) { updateAABB(); }

    Node(const Node& other)
        :aabb_(other.aabb_),
        is_leaf_(other.is_leaf_),
        hittable_buffer_(other.hittable_buffer_),
        hittable_range_(other.hittable_range_) {}

    std::vector<NodePtr>& children() { return children_; }
    // std::vector<Hittable>& hittables() { return hittables_; }
    const AABB& boundingBox() const { return aabb_; }
    void updateAABB()
    {
        aabb_.clear();
        for(size_t idx = hittable_range_[0]; idx < hittable_range_[1]; idx++)
        {
            // std::cout << "aabb: " << hittable_buffer_->at(idx).rigidBody().aabb().str() << std::endl;
            aabb_.extend(hittable_buffer_->at(idx).rigidBody().aabb());
        }
        // std::cout << "aabb: " << aabb_.str() << std::endl;

    }

    Hittable::HitRecordPtr hit(Ray& ray) const
    {
        return closeHit(ray);
    }

    Hittable::HitRecordPtr closeHit(Ray& ray) const
    {
        //std::cout << __FILE__ << ":" << __LINE__ << std::endl;
        // std::cout << "max: " << aabb_.max().T().str()
        //     << ", min: " << aabb_.min().T().str() << std::endl;
        if(!aabb_.hit(ray)) return nullptr;
        // std::cout << __FILE__ << ":" << __LINE__ << std::endl;
        return is_leaf_ ? closeHitLeaf(ray) : closeHitInternalNode(ray);
    }

    void multiHit(const Ray& ray, size_t& hit_count) const
    {
        if(!aabb_.hit(ray)) return;
        return is_leaf_ ? multiHitLeaf(ray, hit_count) : multiHitInternalNode(ray, hit_count);
    }

    void split(size_t max_hittable_num=4, bool verbose=true, size_t children_num=2)
    {
        if(hittable_range_[1] - hittable_range_[0] <= max_hittable_num)
        {
            is_leaf_ = true;
            return;
        }

        is_leaf_ = false;

        children_.push_back(std::make_unique<Node>(*this));
        children_.push_back(std::make_unique<Node>(*this));

        size_t mid = sortInLongestAxis(hittable_buffer_, hittable_range_, aabb_.dim(), verbose);

        children_.front()->hittable_range_[1] = mid;
        children_.back()->hittable_range_[0] = mid;

        for(auto & child: children_)
        {
            child->updateAABB();
            // std::cout << "max: " << child->aabb_.max().T().str()
            // << "min: " << child->aabb_.min().T().str() << std::endl;
            child->split(max_hittable_num, verbose, children_num);
        }
    }

private:
    Hittable::HitRecordPtr closeHitLeaf(Ray& ray) const
    {
        Hittable::HitRecordPtr result = nullptr;
        // std::cout << __FILE__ << ":" << __LINE__ << std::endl;
        // for(auto & hittable : hittables_)
        for(size_t i = hittable_range_[0]; i < hittable_range_[1]; i++)
        {
            auto & hittable = hittable_buffer_->at(i);
            auto p_record = hittable.rigidBody().hit(ray);
            if(!p_record) continue;

            result = std::make_shared<Hittable::HitRecord>(
                hittable.material().attenuation(*p_record),
                // hittable.material().localFrameScatter(ray, *p_record, hittable.rigidBody().pose()),
                hittable.material().scatter(ray, *p_record),
                p_record->t);

            #if 0
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
                std::cout << "ray ori: " << ray.origin().T().str() << "\n"
                << "ray dir: " << ray.direction().T().str() << "\n";
                exit(-1);
            }
            #endif

            ray.tMax() = p_record->t;
        }
        return result;
    }

    Hittable::HitRecordPtr closeHitInternalNode(Ray& ray) const
    {
        Hittable::HitRecordPtr result(nullptr);
        for(auto & child: children_)
        {
            auto p_record = child->hit(ray);
            if(!p_record) continue;
            result = p_record;
            ray.tMax() = p_record->hit_t;
        }
        // std::cout << __FILE__ << ":" << __LINE__ << std::endl;
        return result;
    }

    void multiHitLeaf(const Ray& ray, size_t& hit_count) const
    {
        for(size_t i = hittable_range_[0]; i < hittable_range_[1]; i++)
        {
            std::vector<RigidBody::HitRecordPtr> p_rigid_records;
            hittable_buffer_->at(i).rigidBody().multiHit(ray, p_rigid_records);
            hit_count += p_rigid_records.size();
        }
    }

    void multiHitInternalNode(const Ray& ray, size_t& hit_count) const
    {
        Hittable::HitRecordPtr result(nullptr);
        for(auto & child: children_)
        {
            std::vector<RigidBody::HitRecordPtr> p_rigid_records;
            child->multiHit(ray, hit_count);
        }
    }

private:
    AABB aabb_;
    std::vector<NodePtr> children_;
    bool is_leaf_;
    HittableBufferPtr hittable_buffer_;
    std::array<size_t, 2> hittable_range_;
};

inline size_t sortInLongestAxis(
    HittableBufferPtr buffer,
    const std::array<size_t, 2>& range,
    size_t dim,
    bool verbose=true)
{
    AABB box(dim);
    for(size_t i = range[0]; i < range[1]; i++)
    {
        box.extend(buffer->at(i).rigidBody().aabb());
    }
    size_t target_axis = argMax(box.max() - box.min());
    if(verbose)
        std::cout << "range: [" << range[0] << "," << range[1] << "), axis: " << target_axis << std::endl;
#if 0
    std::array<size_t, 2> p(range);
    p[1] -= 1;
    while(p[0] < p[1])
    {
        if( buffer->at(p[0]).rigidBody().aabb().center(target_axis) >
            buffer->at(p[1]).rigidBody().aabb().center(target_axis))
        {
            Hittable temp(buffer->at(p[0]));
            buffer->at(p[0]) = buffer->at(p[1]);
            buffer->at(p[1]) = temp;
        }
        p[0] ++;
        p[1] --;
    }
    return p[0];
#endif
    sort(buffer->begin() + range[0], buffer->begin() + range[1],
        [&](const Hittable& h1, const Hittable& h2){
            return h1.rigidBody().aabb().center(target_axis)
                < h2.rigidBody().aabb().center(target_axis);});
    return (range[0] + range[1]) / 2;
}

} // namespace bvh


class AcceleratedHitManager: public HitManager
{
public:
    // Hittable::HitRecordPtr hit(const Ray& ray) const = delete;
    Hittable::HitRecordPtr hit(Ray& ray) const { return root_->hit(ray); }
    void setRoot(std::shared_ptr<bvh::Node> root) { root_ = root; }
private:
    std::shared_ptr<bvh::Node> root_;
};

} // namespace rtc




#endif // _BOUNDING_VOLUME_HIERARCHY_H_
