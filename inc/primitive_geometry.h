#if !defined(_PRIMITIVE_GEOMETRY_H)
#define _PRIMITIVE_GEOMETRY_H

#include "linalg.h"
#include "ray.h"

namespace rtc
{
// return Vec: [t, k1, k2, ..., kn]
// intersect = sum(k) < 1 ? true : false
inline Vec intersectEquation(const Mat& primitive, const Ray& ray)
{
    if(!primitive.square())
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    if(primitive.shape(0) != ray.origin().size())
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    // (d BA CA) (t k1 k2).T = OA
    // Ax = b

    Vec b = primitive(Col(0)) - ray.origin();
    Mat mat_a(primitive.shape());
    mat_a.set(Col(0), ray.direction());
    for(size_t i = 1; i < primitive.shape(0); i++)
    {
        mat_a.set(Col(i), primitive(Col(0)) - primitive(Col(i)));
    }
    return qr::solve(mat_a, b);
}

inline bool validIntersect(const Vec& x)
{
    for(size_t i = 1; i < x.size(); i++)
    {
        if(x(i) < 0) return false;
    }
    FloatType sum_k = x.norm(1) - x(0);
    return sum_k < 1;
}

} // namespace rtc



#endif // _PRIMITIVE_GEOMETRY_H
