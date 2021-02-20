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
    mat_a(Col(0)) = ray.direction();
    for(size_t i = 1; i < primitive.shape(0); i++)
    {
        mat_a(Col(i)) = primitive(Col(0)) - primitive(Col(i));
    }
    return qr::solve(mat_a, b);
}

inline bool validIntersect(const Vec& x)
{
    for(size_t i = 1; i < x.size(); i++)
    {
        if(x(i) < 0) return false;
    }
    FloatType sum_k = x.norm(1) - fabs(x(0));
    return sum_k < 1 + eps();
}

//
// triangle: triangle in N-Dimensional space triangle, shape = {N, 3}
// hit_p: hit point in N-Dimensional space triangle, size = N
// triangle_3d: triangle in 2-Dimensional space triangle, shape = {2, 3}
// hit_p_3d: hit point in 2-Dimensional space triangle, size = 2
inline void putTriangleInPlane(
    const Mat& triangle, const Vec& hit_p,
    Mat& triangle_2d, Vec& hit_p_2d)
{
    triangle_2d = Mat({2,3});
    Vec v_ab(triangle(Col(1)) - triangle(Col(0)));
    Vec v_ac(triangle(Col(2)) - triangle(Col(0)));
    Vec dir_ab(v_ab.normalized());
    FloatType l_ab = v_ab.norm();
    // Bx
    triangle_2d(0, 1) = l_ab;
    // Cx, Cy
    triangle_2d(0, 2) = dir_ab.dot(v_ac);
    triangle_2d(1, 2) = (v_ac - triangle_2d(0, 2)* dir_ab).norm();

    hit_p_2d = Vec(2);
    Vec v_ap(hit_p - triangle(Col(0)));
    hit_p_2d(0) = dir_ab.dot(v_ap);
    hit_p_2d(1) = (v_ap - hit_p_2d(0)* dir_ab).norm();
}

template<typename DType>
inline Vector<DType> primitiveNorm(
    const Matrix<DType>& prim, const Ray& ray)
{
    Matrix<DType> vs = prim(Block({},{1, end()})) - prim(Block({},{0, end() - 1}));
    Vector<DType> norm = orthogonalComplement(vs);
    if(norm.dot(ray.direction()) > 0) norm *= -1;
    return norm.normalized();
}

} // namespace rtc



#endif // _PRIMITIVE_GEOMETRY_H
