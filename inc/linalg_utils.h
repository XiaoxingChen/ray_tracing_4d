#if !defined(_LINALG_UTILS_H)
#define _LINALG_UTILS_H

#include "linalg_mat.h"

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
} // namespace rtc
#endif // _LINALG_UTILS_H
