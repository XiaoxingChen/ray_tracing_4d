#if !defined(_PRIMITIVE_MESH_TREE_)
#define _PRIMITIVE_MESH_TREE_

#include "bounding_volume_hierarchy.h"
#include <stack>


namespace rtc
{

namespace bvh2
{

struct Node
{
    Node(size_t dim): aabb(dim), is_leaf(false){}
    AABB aabb;
    std::vector<size_t> children_index_buffer;
    std::vector<size_t> primitive_index_buffer;
    bool is_leaf;
};

Mat getPrimitive(const Mat& vertex_buffer, const Matrix<size_t>& vertex_index_buffer, size_t primitive_index)
{
    size_t dim = vertex_buffer.shape(0);
    size_t vertex_per_primitive = vertex_index_buffer.shape(0);
    Mat ret({dim, vertex_per_primitive});
    for(size_t i = 0; i < vertex_per_primitive; i++)
    {
        size_t vertex_index = vertex_index_buffer(i, primitive_index);
        ret(Col(i)) = vertex_buffer(Col(vertex_index));
    }
    return ret;
}

class PrimitiveMeshTree
{
public:
    void build(size_t primitive_per_leaf=4, bool verbose=true);
    size_t dim() const { return position_.size(); }

    PrimitiveMeshTree(const Mat& vertex_buffer, const Matrix<size_t>& vertex_index_buffer)
        :position_(vertex_buffer.shape(0)), orientation_(vertex_buffer.shape(0)),
        vertex_buffer_(vertex_buffer), vertex_index_buffer_(vertex_index_buffer)
        {}

    Mat primitive(size_t idx) const { return getPrimitive(vertex_buffer_, vertex_index_buffer_, idx); }

    size_t multiHit(const Ray& ray) const;

private:
    Vec position_;
    Rotation orientation_;

    Mat vertex_buffer_;
    Matrix<size_t> vertex_index_buffer_;
    std::vector<bvh2::Node> node_buffer_;

};

struct RangeNode
{
    size_t node_idx;
    std::array<size_t, 2> range;
};

size_t treeNodeRequirement(size_t leaf_num, size_t child_num=2)
{
    size_t height = static_cast<size_t>(1. + log(leaf_num) / log(child_num)) + 1;
    if(child_num != 2)
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    return (1 << height);
}

inline void PrimitiveMeshTree::build(size_t primitive_per_leaf, bool verbose)
{
    std::vector<size_t> primitive_index_buffer(vertex_index_buffer_.shape(1));
    for(size_t i = 0; i < primitive_index_buffer.size(); i++) primitive_index_buffer.at(i) = i;

    {
        size_t require = treeNodeRequirement(primitive_index_buffer.size() / primitive_per_leaf + 1);
        node_buffer_.clear();
        node_buffer_.reserve(require);
    }

    std::stack<RangeNode> stk;

    stk.push({node_buffer_.size(), {0, primitive_index_buffer.size()}});
    node_buffer_.push_back(Node(dim()));


    while (!stk.empty())
    {
        auto target = stk.top();
        stk.pop();

        auto& target_node = node_buffer_.at(target.node_idx);

        //
        // eliminate leaf node
        if(target.range[1] - target.range[0] <= primitive_per_leaf)
        {
            for(size_t prim_idx = target.range[0]; prim_idx < target.range[1]; prim_idx++)
            {
                target_node.primitive_index_buffer.push_back(prim_idx);
                target_node.aabb.extend( primitive(prim_idx) );
            }

            target_node.is_leaf = true;
            continue;
        }

        //
        // deal with internal node
        //

        //
        // calculate AABB
        for(size_t i = target.range[0]; i < target.range[1]; i++)
        {
            const size_t& prim_idx = primitive_index_buffer.at(i);
            target_node.aabb.extend(primitive(prim_idx));
        }

        //
        // find the longest axis
        size_t target_axis = argMax(target_node.aabb.max() - target_node.aabb.min());
        if(verbose)
            std::cout << "range: [" << target.range[0] << "," << target.range[1] << "), axis: " << target_axis << std::endl;

        //
        // sort along the target axis
        std::sort(
            primitive_index_buffer.begin() + target.range[0],
            primitive_index_buffer.begin() + target.range[1],
            [&](const size_t& prim_idx1, const size_t& prim_idx2)
            {
                Mat prim1 = primitive(prim_idx1);
                Mat prim2 = primitive(prim_idx2);

                return prim1(Col(target_axis)).asVector().sum() < prim2(Col(target_axis)).asVector().sum();
            });

        // create children
        size_t mid = (target.range[1] + target.range[0]) / 2;

        stk.push({node_buffer_.size(), {target.range[0], mid}});
        target_node.children_index_buffer.push_back(node_buffer_.size());
        node_buffer_.push_back(Node(dim()));

        stk.push({node_buffer_.size(), {mid, target.range[1]}});
        target_node.children_index_buffer.push_back(node_buffer_.size());
        node_buffer_.push_back(Node(dim()));

    }
}

inline size_t PrimitiveMeshTree::multiHit(const Ray& ray) const
{
    std::stack<size_t> node_idx_stk;

    if(!node_buffer_.at(0).aabb.hit(ray))
        return 0;
    node_idx_stk.push(0);

    size_t hit_cnt = 0;
    while(! node_idx_stk.empty())
    {
        const Node& target_node = node_buffer_.at(node_idx_stk.top());
        node_idx_stk.pop();

        //
        // deal with leaf node
        if(target_node.is_leaf)
        {
            for(const auto& prim_idx: target_node.primitive_index_buffer)
            {
                if(validIntersect (
                    intersectEquation( primitive(prim_idx), ray)))
                    hit_cnt++;
            }
            continue;
        }

        //
        // deal with internal node
        for(const auto& child_idx: target_node.children_index_buffer)
        {
            if(node_buffer_.at(child_idx).aabb.hit(ray))
                node_idx_stk.push(child_idx);
        }

    }

    return hit_cnt;
}


} // namespace bvh

} // namespace rtc




#endif // _PRIMITIVE_MESH_TREE_
