#if !defined(_LINALG_VEC_H)
#define _LINALG_VEC_H

#include <vector>
#include <array>
#include <functional>
#include <numeric>
#include <array>
#include <string>
#include <assert.h>
#include <math.h>
#include "base_type.h"
#include "linalg_mat.h"


namespace rtc
{

class Pixel;

class Vec: public Mat
{
public:
    Vec(size_t size): Mat({size, 1}){}
    Vec(const std::vector<FloatType>& v): Mat({v.size(), 1}, v){}
    // Vec(std::vector<FloatType>&& v): Mat({v.size(), 1}, std::move(v)){}
    size_t size() const { return data_.size(); }

    FloatType& operator () (size_t i) { return data_.at(i); }
    const FloatType& operator () (size_t i) const { return data_.at(i); }

    FloatType dot(const Vec& rhs) const
    {
        if(size() != rhs.size())
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
        FloatType sum(0);
        for(int i = 0; i < size(); i++) sum += data_.at(i) * rhs.data_.at(i);
        return sum;
    }

    static Mat zeros(size_t n) { return Vec(n); }

    static Mat ones(size_t n) { return Vec(n) + 1; }

    operator Pixel() const;
};

class UnitVec: public Vec
{
public:
    UnitVec(size_t size): Vec(size) {data_.at(0) = 1;}
    UnitVec(const Vec& v): Vec(v) {this->normalize();}
    UnitVec(const std::vector<FloatType>& v): Vec(v) {this->normalize();}

    Mat& operator *= (FloatType scalar) = delete;
    Mat& operator += (FloatType scalar) = delete;
    Mat& operator -= (FloatType scalar) = delete;
    Mat& operator *= (const Mat& rhs) = delete;
    Mat& operator += (const Mat& rhs) = delete;
    Mat& operator -= (const Mat& rhs) = delete;

    const FloatType& operator () (size_t i) const { return data_.at(i); }

    // FloatType& operator () (size_t i) {}
    // FloatType& operator () (size_t i, size_t j) {}

private:
    using Mat::normalized;
    using Mat::normalize;
};

using VecIn = const Vec&;
using UnitVecIn = const UnitVec&;

inline Mat::operator rtc::Vec() const
{
    if(shape(1) != 1) throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    return Vec(data_);
}

class Pixel: public Vec
{
public:
    using InitialType = std::array<FloatType, 3>;
    Pixel(const InitialType& v): Vec(std::vector<FloatType>(v.begin(), v.end())){}
    Pixel(const Vec& v): Vec(v){}
    Pixel(): Vec(3){}

    const float_t & r() const {return data_.at(0);}
    const float_t & g() const {return data_.at(1);}
    const float_t & b() const {return data_.at(2);}

    int rU8() const {return r() < 0 ? 0 : r() > 1 ? 255 : r() * 255.99;}
    int gU8() const {return g() < 0 ? 0 : g() > 1 ? 255 : g() * 255.99;}
    int bU8() const {return b() < 0 ? 0 : b() > 1 ? 255 : b() * 255.99;}

    static Pixel black() {return Pixel();}
    static Pixel white() {return Pixel({1,1,1});}

private:

};

inline Vec::operator Pixel() const {return Pixel(*this);}

} // namespace rtc


#endif // _LINALG_VEC_H
