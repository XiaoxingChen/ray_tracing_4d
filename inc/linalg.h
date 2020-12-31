#if !defined(_LINALG_H_)
#define _LINALG_H_

#include <vector>
#include <functional>
#include <numeric>
#include <array>
#include <string>
#include <assert.h>
#include <math.h>
#include "base_type.h"

namespace rtc
{
class Vec;
class Mat
{
public:
    using Shape = std::array<size_t, 2>;

    size_t shape(uint8_t i) const {return shape_.at(i);}
    // const Shape shape;
    bool square() const {return shape(0) == shape(1);}
    bool rowMajor() const {return row_major_;}

    Mat(const Shape& _shape, const std::vector<FloatType>& data={})
        :shape_(_shape), row_major_(true)
    {
        if(data.size() == 0)
        {
            data_.resize(shape(0) * shape(1), 0.);
            return;
        }
        if(shape(0) * shape(1) != data.size())
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
        data_ = data;
    }

    // Mat(Mat && mat): shape(mat.shape), data_(std::move(mat.data_)), row_major_(mat.row_major_){}

    static Mat Identity(size_t n)
    {
        Mat mat({n,n});
        for(int i = 0; i < n; i++) mat(i,i) = 1;
        return mat;
    }

    FloatType& operator () (size_t i, size_t j)
    {
        if(row_major_)
            return data_.at(i * shape(1) + j);
        else
            return data_.at(j * shape(0) + i);
    }

    const FloatType& operator () (size_t i, size_t j) const
    {
        if(row_major_)
            return data_.at(i * shape(1) + j);
        else
            return data_.at(j * shape(0) + i);
    }

    template<typename Op>
    Mat& opEqual(FloatType scalar, Op f) { for(auto & v: data_) v = f(v, scalar); return *this;}

    template<typename Op>
    Mat& opEqual(const Mat& rhs, Op f)
    {
        if(shape_ != rhs.shape_)
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
        for(int i = 0; i < data_.size(); i++)
            data_.at(i) = f(data_.at(i), rhs.data_.at(i));
        return *this;
    }

    Mat& operator *= (FloatType scalar)  { return opEqual(scalar, std::multiplies<FloatType>()); }
    Mat& operator += (FloatType scalar)  { return opEqual(scalar, std::plus<FloatType>()); }
    Mat& operator -= (FloatType scalar)  { return opEqual(scalar, std::minus<FloatType>()); }

    Mat operator * (FloatType scalar) const { return Mat(*this) *= scalar; }
    Mat operator + (FloatType scalar) const { return Mat(*this) += scalar; }
    Mat operator - (FloatType scalar) const { return Mat(*this) -= scalar; }

    Mat& operator *= (const Mat& rhs)  { return opEqual(rhs, std::multiplies<FloatType>()); }
    Mat& operator += (const Mat& rhs)  { return opEqual(rhs, std::plus<FloatType>()); }
    Mat& operator -= (const Mat& rhs)  { return opEqual(rhs, std::minus<FloatType>()); }

    Mat operator * (const Mat& rhs) const { return Mat(*this) *= rhs; }
    Mat operator + (const Mat& rhs) const { return Mat(*this) += rhs; }
    Mat operator - (const Mat& rhs) const { return Mat(*this) -= rhs; }

    operator Vec() const;
    // operator Vec() const { return Vec(data_); }

    FloatType norm(int8_t p = 'f') const
    {
        if('f' == p || 2 == p)
        {
            FloatType sum2(0);
            for(auto & v : data_) sum2 += v*v;
            return sqrt(sum2);
        }
        if(1 == p)
        {
            FloatType max_abs_sum(0);
            for(int j = 0; j < shape(1); j++)
            {
                FloatType abs_sum(0);
                for(int i = 0; i < shape(0); i++) abs_sum += fabs((*this)(i,j));
                max_abs_sum = std::max(max_abs_sum, abs_sum);
            }
            return max_abs_sum;
        }

        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
        return 0.;
    }

    const Mat& normalize(int8_t p = 2)
    {
        FloatType n = norm(p);
        if(n < eps()) throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
        for(auto & v : data_) v /= n;
        return *this;
    }

    Mat normalized() const { return Mat(*this).normalize(); }

    Mat T() const
    {
        Mat ret({shape(1), shape(0)}, data_);
        ret.row_major_ = !row_major_;
        return ret;
    }

    Mat dot(const Mat& rhs) const { return matmul(rhs); }

    Mat matmul(const Mat& rhs) const
    {
        const Mat& lhs(*this);
        if(lhs.shape(1) != rhs.shape(0))
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
        Mat ret({lhs.shape(0), rhs.shape(1)});
        for(int i = 0; i < ret.shape(0); i++)
        {
            for(int j = 0; j < ret.shape(1); j++)
            {
                ret(i,j) = 0;
                for(int k = 0; k < lhs.shape(1); k++)
                {
                    ret(i,j) += lhs(i, k) * rhs(k, j);
                }
            }
        }
        return ret;
    }

    std::string str() const
    {
        std::string ret;
        for(int i = 0; i < shape(0); i++)
        {
            for(int j = 0; j < shape(1); j++)
            {
                ret += (std::to_string((*this) (i,j)) + " ");
            }
            ret += "\n";
        }
        return ret;
    }

protected:
    Shape shape_;
    std::vector<FloatType> data_;
    bool row_major_;
};

class Vec: public Mat
{
public:
    Vec(size_t size): Mat({size, 1}){}
    Vec(const std::vector<FloatType>& v): Mat({v.size(), 1}, v){}
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


} // namespace rtc


#endif // _LINALG_H_
