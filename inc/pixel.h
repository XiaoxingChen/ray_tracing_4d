#if !defined(_PIXEL_H_)
#define _PIXEL_H_

#include <array>
#include <cmath>
#include "base_type.h"
#include "accessor.h"


namespace rtc
{

class Pixel: public LinearData<Pixel, FloatType>
{
public:
    static const size_t NChannel = 3;
    using InitialType = std::array<FloatType, NChannel>;
    using ThisType = Pixel;
    Pixel(const InitialType& v): data_(v){}
    Pixel(){ traverse([&](size_t i){(*this)(i) = 0;}); }

    const FloatType& operator () (size_t i) const { return data_.at(i); }
    FloatType& operator () (size_t i) { return data_.at(i); }

    const FloatType & r() const {return (*this)(0);}
    const FloatType & g() const {return (*this)(1);}
    const FloatType & b() const {return (*this)(2);}

    int rU8() const {return r() < 0 ? 0 : r() > 1 ? 255 : r() * 255.99;}
    int gU8() const {return g() < 0 ? 0 : g() > 1 ? 255 : g() * 255.99;}
    int bU8() const {return b() < 0 ? 0 : b() > 1 ? 255 : b() * 255.99;}

    static Pixel black() {return Pixel();}
    static Pixel white() {return Pixel({1,1,1});}
    size_t size() const {return data_.size();}

private:
    std::array<FloatType, NChannel> data_;

};

inline Pixel operator * (const FloatType& scalar, const Pixel& rhs ) {return rhs * scalar;}

} // namespace rtc

#endif // _PIXEL_H_
