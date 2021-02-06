#if !defined(_INTERPOLATION_H_)
#define _INTERPOLATION_H_

#include "linalg_vec.h"


namespace rtc
{

namespace interp
{

//
// https://en.wikipedia.org/wiki/Bilinear_interpolation#Unit_square
//
// PType: should be either float or double
// DType: can be any type that overloaded with DType::operator*(PType scalar)
//      for example: PixelRGB, Complex, Quaternion
// square: matrix with shape {2,2}
//      | f(0,0), f(0,1) |
//      | f(1,0), f(1,1) |
// pos: (x,y) position
template<typename PType, typename DType>
DType bilinearUnitSquare(const Vector<PType>& pos, const Matrix<DType>& square)
{
    Vector<PType> vx(2, {1 - pos(0), pos(0)});
    Vector<PType> vy(2, {1 - pos(1), pos(1)});
    return vx.T().matmul(square).matmul(vy);
}

//
// https://codeplea.com/triangular-interpolation
//
// PType: should be either float or double
// DType: can be any type that overloaded with DType::operator*(PType scalar)
//      for example: PixelRGB, Complex, Quaternion
// v: vertex matrix with shape {2, 3}, dim is the dimension of space
template<typename PType, typename DType>
DType triangular(const Vector<PType>& pos, const Matrix<PType>& v, const Vector<DType>& val)
{
    Vector<PType> weight(3);
    constexpr size_t X = 0;
    constexpr size_t Y = 1;
    PType dom = (v(Y,1) - v(Y,2)) * (v(X,0) - v(X,2)) + (v(X,2) - v(X,1)) * (v(Y,0) - v(Y,2));
    weight(0) = (v(Y,1) - v(Y,2)) * (pos(X) - v(X,2)) + (v(X,2) - v(X,1)) * (pos(Y) - v(Y,2)) / dom;
    weight(1) = (v(Y,2) - v(Y,0)) * (pos(X) - v(X,2)) + (v(X,0) - v(X,2)) * (pos(Y) - v(Y,2)) / dom;
    weight(2) = 0 - weight(0) - weight(1);

    return weight.T().matmul(val)(0,0);
}


} // namespace interp
} // namespace rtc




#endif // _INTERPOLATION_H_
