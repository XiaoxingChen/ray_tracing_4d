#if !defined(_TEST_BVH_H)
#define _TEST_BVH_H

#include "bounding_volume_hierarchy.h"

using namespace rtc;

inline std::shared_ptr<Mat> createTriangleBand(
    size_t half_n,
    std::vector<std::vector< size_t>>& indices)
{
    size_t dim = 3;
    std::shared_ptr<Mat> ret(new Mat({half_n * 2, dim}));
    Mat& vertices = *ret;
    indices.clear();
    // std::vector<std::vector< size_t>> indices;
    for(size_t n = 0; n < half_n; n++)
    {
        size_t i = 2*n;
        vertices.set(Col(i), Vec({static_cast<FloatType>(i), 0, 0}));
        vertices.set(Col(i + 1), Vec({static_cast<FloatType>(i), 1, 0}));

        if(i < half_n - 1)
        {
            indices.push_back({i, i + 3, i + 1});
            indices.push_back({i, i + 2, i + 3});
        }
    }
    return ret;
}

inline void testBuildTree()
{
    size_t dim = 3;
    std::vector<std::vector< size_t>> index_buffer;
    auto vertex_buffer = createTriangleBand(40, index_buffer);

    bvh::NodePtr root = std::make_unique<bvh::Node>(dim);
}

#endif // _TEST_BVH_H
