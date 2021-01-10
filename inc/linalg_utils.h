#if !defined(_LINALG_UTILS_H)
#define _LINALG_UTILS_H

#include "linalg_mat.h"
#include <algorithm>

namespace rtc
{


inline Mat orthogonalComplement(const Mat& vs)
{
    if(vs.shape(0) == 2 && vs.shape(1) == 3)
    {
        Mat ret({1,3});
        ret(0,0) = vs(0,1) * vs(1,2) - vs(0,2) * vs(1,1);
        ret(0,1) = -vs(0,0) * vs(1,2) + vs(1,0) * vs(0,2);
        ret(0,2) = vs(0,0) * vs(1,1) - vs(1,0) * vs(0,1);
        return ret;
    }
    if(vs.shape(0) == 3 && vs.shape(1) == 2)
    {
        return orthogonalComplement(vs.T()).T();
    }

    if(vs.shape(0) == 1 && vs.shape(1) == 3)
    {

        Mat random_plane({2,3});
        for(size_t i = 0; i < 3; i++)
        {
            random_plane(0, i) = vs(0, i);
            random_plane(1, i) = vs(0, i) + i + 1;
        }
        Mat complement = orthogonalComplement(random_plane);
        Mat ret({2,3});
        for(size_t i = 0; i < 3; i++)
        {
            ret(0, i) = complement(0, i);
            random_plane(1, i) = complement(0, i);
        }
        complement = orthogonalComplement(random_plane);
        for(size_t i = 0; i < 3; i++)
        {
            ret(1, i) = complement(0, i);
        }
        return ret;
    }

    if(vs.shape(0) == 3 && vs.shape(1) == 1)
    {
        return orthogonalComplement(vs.T()).T();
    }
    throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    return Mat({1,1});
}

inline size_t argMax(const Vec& v)
{
    size_t idx_max(0);
    FloatType val_max(v(0));
    for(size_t i = 0; i < v.size(); i++)
    {
        if(v(i) > val_max) idx_max = i;
    }
    return idx_max;
}

inline std::vector<size_t> argSort(const Vec& v)
{
    std::vector<size_t> indices(v.size());
    sort(indices.begin(), indices.end(),
        [&v](size_t i1, size_t i2){ return v(i1) < v(i2); });
}

} // namespace rtc
#endif // _LINALG_UTILS_H
