#if !defined(_TEST_BVH_H)
#define _TEST_BVH_H

#include "bounding_volume_hierarchy.h"
#include "primitive_mesh_tree.h"

using namespace rtc;

inline std::shared_ptr<Mat> createTriangleBand(
    size_t half_n,
    Matrix<size_t>& indices)
{
    size_t dim = 3;
    std::shared_ptr<Mat> ret(new Mat({dim, half_n * 2}));
    Mat& vertices = *ret;
    // indices.clear();
    indices = Matrix<size_t>({3, half_n});
    // std::vector<std::vector< size_t>> indices;
    for(size_t n = 0; n < half_n; n++)
    {
        size_t i = 2*n;
        vertices(Col(i)) = Vec({static_cast<FloatType>(i), 0, 0});
        vertices(Col(i + 1)) = Vec({static_cast<FloatType>(i), 1, 0});

        if(i < half_n - 1)
        {
            indices(Col(i)) = Vector<size_t>({i, i + 3, i + 1});
            indices(Col(i+1)) = Vector<size_t>({i, i + 2, i + 3});
            // indices.push_back({i, i + 3, i + 1});
            // indices.push_back({i, i + 2, i + 3});
        }
    }
    return ret;
}

inline void testSort()
{
    size_t dim = 3;
    Matrix<size_t> index_buffer;

    size_t triangle_num = 40;
    auto vertex_buffer = createTriangleBand(40, index_buffer);
    // std::cout << index_buffer.T().str() << std::endl;
    HittableBufferPtr hittable_buffer = std::make_shared<HittableBuffer>();
    for(size_t i = 0; i < index_buffer.shape(1); i++)
    {
        hittable_buffer->push_back(
            Hittable(
                RigidBody::createPolygonPrimitive(vertex_buffer, index_buffer(Col(i)).asVector()),
                Material::choose(Material::METAL)));
    }

    size_t mid = bvh::sortInLongestAxis(hittable_buffer, {0, hittable_buffer->size()}, 3);
    if(mid != triangle_num / 2)
    {
        std::cout << "mid: " << mid << std::endl;
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }


}

inline void testBuildTree()
{
    size_t dim = 3;
    Matrix<size_t> index_buffer;

    size_t triangle_num = 40;
    auto vertex_buffer = createTriangleBand(40, index_buffer);
    HittableBufferPtr hittable_buffer = std::make_shared<HittableBuffer>();
    for(size_t i = 0; i < index_buffer.shape(1); i++)
    {
        hittable_buffer->push_back(
            Hittable(
                RigidBody::createPolygonPrimitive(vertex_buffer, index_buffer(Col(i)).asVector()),
                Material::choose(Material::METAL)));
    }

    auto root(
        std::unique_ptr<bvh::Node>(
            new bvh::Node ((dim), hittable_buffer, {0, hittable_buffer->size()})
        ));
    root->split(4, true);

}

inline void testBuildTree2()
{
    std::cout << "testBuildTree2()" << std::endl;
    size_t dim = 3;
    Matrix<size_t> index_buffer;

    size_t triangle_num = 40;
    auto vertex_buffer = createTriangleBand(40, index_buffer);

    bvh2::PrimitiveMeshTree tree(*vertex_buffer, index_buffer);
    tree.build(4, true);
}

void testBvh()
{
    testSort();
    testBuildTree();
    testBuildTree2();
}

#endif // _TEST_BVH_H
