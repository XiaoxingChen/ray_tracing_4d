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

class PrimitiveMeshTree
{
public:
    void build(size_t primitive_per_leaf=4, bool verbose=true);
    size_t dim() const { return position_.size(); }

    PrimitiveMeshTree(const Mat& vertex_buffer, const Matrix<size_t>& vertex_index_buffer)
        :position_(vertex_buffer.shape(0)), orientation_(vertex_buffer.shape(0)),
        vertex_buffer_(vertex_buffer), vertex_index_buffer_(vertex_index_buffer)
        {}

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

void PrimitiveMeshTree::build(size_t primitive_per_leaf, bool verbose)
{
    std::vector<size_t> primitive_index_buffer(vertex_index_buffer_.shape(1));
    for(size_t i = 0; i < primitive_index_buffer.size(); i++) primitive_index_buffer.at(i) = i;

    std::stack<RangeNode> stk;

    stk.push({node_buffer_.size(), {0, primitive_index_buffer.size()}});
    node_buffer_.push_back(Node(dim()));

    while (!stk.empty())
    {
        auto target = stk.top();
        stk.pop();

        //
        // eliminate leaf node
        if(target.range[1] - target.range[0] <= primitive_per_leaf)
        {
            for(size_t prim_idx = target.range[0]; prim_idx < target.range[1]; prim_idx++)
                node_buffer_.at(target.node_idx).primitive_index_buffer.push_back(prim_idx);
            node_buffer_.at(target.node_idx).is_leaf = true;
            continue;
        }

        //
        // deal with internal node
        //

        //
        // calculate AABB
        auto & target_aabb = node_buffer_.at(target.node_idx).aabb;
        for(size_t i = target.range[0]; i < target.range[1]; i++)
        {
            const size_t& prim_idx = primitive_index_buffer.at(i);
            target_aabb.extend(getPrimitive(vertex_buffer_, vertex_index_buffer_, prim_idx));
        }

        //
        // find the longest axis
        size_t target_axis = argMax(target_aabb.max() - target_aabb.min());
        if(verbose)
            std::cout << "range: [" << target.range[0] << "," << target.range[1] << "), axis: " << target_axis << std::endl;

        //
        // sort along the target axis
        std::sort(
            primitive_index_buffer.begin() + target.range[0],
            primitive_index_buffer.begin() + target.range[1],
            [&](const size_t& prim_idx1, const size_t& prim_idx2)
            {
                Mat prim1 = getPrimitive(vertex_buffer_, vertex_index_buffer_, prim_idx1);
                Mat prim2 = getPrimitive(vertex_buffer_, vertex_index_buffer_, prim_idx2);

                return prim1(Col(target_axis)).asVector().sum() < prim2(Col(target_axis)).asVector().sum();
            });

        // create children
        size_t mid = (target.range[1] + target.range[0]) / 2;

        stk.push({node_buffer_.size(), {target.range[0], mid}});
        node_buffer_.push_back(Node(dim()));

        stk.push({node_buffer_.size(), {mid, target.range[1]}});
        node_buffer_.push_back(Node(dim()));

    }
}


} // namespace bvh

} // namespace rtc




#endif // _PRIMITIVE_MESH_TREE_
