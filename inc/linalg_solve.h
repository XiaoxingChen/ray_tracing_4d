#if !defined(_LINALG_SOLVE_H)
#define _LINALG_SOLVE_H

#include "linalg_vec.h"

namespace rtc
{

inline Vec solveLUTriangle(const Mat& mat, const Vec& b, bool l_tri)
{
    if(!mat.square())
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    if(mat.shape(0) != b.size())
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    size_t idx_start = l_tri ? 0 : b.size() - 1;
    int step = l_tri ? 1 : - 1;
    Vec x(Vec::zeros(b.size()));

    for(size_t i = idx_start, _1 = 0; _1 < b.size(); _1 ++, i += step)
    {
        FloatType off_diag_sum = 0;
        for(size_t j = idx_start, _2 = 0; _2 < _1; _2 ++, j += step)
        {
            off_diag_sum += mat(i, j) * x(j);
        }
        x(i) = (b(i) - off_diag_sum) / mat(i,i);
    }
    return x;
}


inline Vec solveLowerTriangle(const Mat& lower, const Vec& b)
{
    return solveLUTriangle(lower, b, 1);
}

inline Vec solveUpperTriangle(const Mat& upper, const Vec& b)
{
    return solveLUTriangle(upper, b, 0);
}

namespace qr
{
inline Vec project(UnitVecIn u, const Vec& a)
{
    return u.dot(a) * u;
}

inline Mat calcMatQ(const Mat& mat)
{
    //reference: https://en.wikipedia.org/wiki/QR_decomposition
    if(!mat.square())
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    Mat mat_u({mat.shape(0), mat.shape(1)});
    for(size_t i = 0; i < mat.shape(0); i++)
    {
        Vec a_i(mat(Col(i)));
        mat_u.set(Col(i), a_i);
        for(size_t j = 0; j < i; j++)
        {
            mat_u.set(Col(i), mat_u(Col(i)) - project(mat_u(Col(j)), a_i));
        }
        mat_u.set(Col(i), static_cast<UnitVec>(mat_u(Col(i))));
    }
    return mat_u;
}

inline Vec solve(const Mat& mat_a, const Vec& b)
{
    Mat mat_q(calcMatQ(mat_a));
    Mat mat_r(mat_q.T().matmul(mat_a));
    Vec x(solveUpperTriangle(mat_r, mat_q.T().dot(b)));
    return x;
}
} // namespace qr


} // namespace rtc



#endif // _LINALG_SOLVE_H
