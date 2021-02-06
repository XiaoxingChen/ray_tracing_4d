#if !defined(_PIXEL_H_)
#define _PIXEL_H_

#include <array>
#include <cmath>
#include "base_type.h"
#include "accessor.h"


namespace rtc
{

template<typename DType>
inline int quantizeToU8(DType val)
{
    return val < 0 ? 0 : val > 1 ? 255 : val * 255.99;
}

template<typename DeriveType>
class RGBA_F32
{
public:
    using DType = float;
    DeriveType* deriveThis() {return reinterpret_cast<DeriveType*>(this);}
    const DeriveType* deriveThis() const {return reinterpret_cast<const DeriveType*>(this);}
    const DType& at(size_t i) const {return (*deriveThis())(i);}

    const FloatType & r() const {return (*deriveThis())(0);}
    const FloatType & g() const {return (*deriveThis())(1);}
    const FloatType & b() const {return (*deriveThis())(2);}
    const FloatType & a() const {return deriveThis()->size() > 2 ? (*deriveThis())(3) : 0.;}

    int rU8() const { return quantizeToU8<DType>(r()); }
    int gU8() const { return quantizeToU8<DType>(g()); }
    int bU8() const { return quantizeToU8<DType>(b()); }
    int aU8() const { return quantizeToU8<DType>(a()); }
};

template<size_t NChannel>
class PixelF32: public LinearData<PixelF32<NChannel>, float>, public RGBA_F32<PixelF32<NChannel>>
{
public:
    // static const size_t NChannel = 3;
    using InitialType = std::array<FloatType, NChannel>;
    using ThisType = PixelF32;
    using DType = float;
    using BaseType0 = LinearData<PixelF32<NChannel>, float>;
    PixelF32(const InitialType& v): data_(v){}
    PixelF32(){ BaseType0::traverse([&](size_t i){(*this)(i) = 0;}); }

    const DType& operator () (size_t i) const { return data_.at(i); }
    DType& operator () (size_t i) { return data_.at(i); }

    static ThisType black() {return ThisType();}
    static ThisType white() {return ThisType() + 1;}
    static constexpr size_t size() {return NChannel;}

private:
    std::array<DType, NChannel> data_;
};

using Pixel = PixelF32<3>;
using Pixel4 = PixelF32<4>;


} // namespace rtc

#endif // _PIXEL_H_
