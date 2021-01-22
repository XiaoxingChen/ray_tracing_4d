#if !defined(_LINALG_MAT_H)
#define _LINALG_MAT_H

#include <vector>
#include <functional>
#include <numeric>
#include <array>
#include <string>
#include <assert.h>
#include <math.h>
#include <sstream>
#include <iomanip>

#include "base_type.h"



namespace rtc
{

size_t indexConvert2D(size_t i, size_t j, bool major, size_t shape_i, size_t shape_j);
inline std::string to_string(const FloatType& v, size_t prec=6)
{
    std::stringstream stream;
    stream << std::fixed << std::setprecision(prec) << v;
    return stream.str();
}

class Vec;
class Block;
class MatRef;

class Mat
{
public:
    using Shape = std::array<size_t, 2>;
    static const bool ROW = 0;
    static const bool COL = 1;
    using ThisType = Mat;

    //
    // interfaces
    virtual size_t shape(uint8_t i) const {return shape_.at(i);}
    virtual const Shape& shape() const {return shape_;}
    bool square() const {return shape(0) == shape(1);}
    bool majorAxis() const {return major_;}
    bool& majorAxis() {return major_;}

    //
    // constructors
    Mat():shape_({0,0}), data_(), major_(ROW) {}

    Mat(const Shape& _shape, const std::vector<FloatType>& data={}, bool major=ROW)
        :shape_(_shape), data_(_shape[0] * _shape[1], 0), major_(major)
    {
        if(data.size() == 0)  return;
        if(shape(0) * shape(1) != data.size())
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
        data_ = data;
    }

    Mat(const ThisType& rhs)
        :shape_(rhs.shape()), data_(rhs.shape(0) * rhs.shape(1)), major_(rhs.majorAxis())
    {
        (*this) = rhs;
    }

    virtual void operator = (const ThisType& rhs)
    {
        if(this->shape() == Shape{0,0}) shape_ = rhs.shape();
        traverse([&](size_t i, size_t j){ (*this)(i,j) = rhs(i,j); });
    }

    //
    // static methods
    //
    static ThisType zeros(const Shape& _shape) { return ThisType(_shape); }
    static ThisType ones(const Shape& _shape) { return ThisType(_shape) + 1; }
    static ThisType Identity(size_t n)
    {
        ThisType mat({n,n});
        for(int i = 0; i < n; i++) mat(i,i) = 1;
        return mat;
    }

    //
    // basic accessor
    //
    virtual const FloatType& operator () (size_t i, size_t j) const
    {
        return data_.at(
            indexConvert2D(i,j,majorAxis(), shape(0), shape(1)));
    }

    virtual FloatType& operator () (size_t i, size_t j) { return const_cast<FloatType&>(static_cast<const ThisType&>(*this)(i,j)); }

    template<typename Op>
    void traverse(Op f) const
    {
        for(size_t i = 0; i < shape(0); i++)
            for(size_t j = 0; j < shape(1); j++)
                f(i, j);
    }

    //
    // arithmetic operators
    //
    ThisType& operator *= (FloatType scalar)  { traverse([&](size_t i, size_t j){(*this)(i,j) *= scalar;}); return *this;}
    ThisType& operator += (FloatType scalar)  { traverse([&](size_t i, size_t j){(*this)(i,j) += scalar;}); return *this;}
    ThisType& operator -= (FloatType scalar)  { traverse([&](size_t i, size_t j){(*this)(i,j) -= scalar;}); return *this;}

    ThisType operator * (FloatType scalar) const { return ThisType(*this) *= scalar; }
    ThisType operator + (FloatType scalar) const { return ThisType(*this) += scalar; }
    ThisType operator - (FloatType scalar) const { return ThisType(*this) -= scalar; }

    ThisType& operator *= (const ThisType& rhs)  { traverse([&](size_t i, size_t j){(*this)(i,j) *= rhs(i,j);}); return *this;}
    ThisType& operator += (const ThisType& rhs)  { traverse([&](size_t i, size_t j){(*this)(i,j) += rhs(i,j);}); return *this;}
    ThisType& operator -= (const ThisType& rhs)  { traverse([&](size_t i, size_t j){(*this)(i,j) -= rhs(i,j);}); return *this;}

    ThisType operator * (const ThisType& rhs) const { return ThisType(*this) *= rhs; }
    ThisType operator + (const ThisType& rhs) const { return ThisType(*this) += rhs; }
    ThisType operator - (const ThisType& rhs) const { return ThisType(*this) -= rhs; }

    ThisType operator -() const        { return ThisType(*this) *= -1;}

    FloatType det() const;
    FloatType trace() const
    {
        if(!square()) throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
        FloatType ret(0);
        for(size_t i = 0; i < shape(0); i++) ret += (*this)(i,i);
        return ret;
    }

    FloatType norm(uint8_t p = 2) const;
    FloatType norm(const std::string& p) const
    {
        if(p == std::string("F")) return norm(2);
        if(p == std::string("inf")) return norm(255);
        return 0;
    }

    const ThisType& normalize(uint8_t p = 2)
    {
        FloatType n = norm(p);
        if(n < eps()*eps()) { return *this; }
        return ((*this) *= (1./n));
    }

    ThisType normalized() const { return ThisType(*this).normalize(); }

    virtual const MatRef T() const;
    virtual MatRef T();

    ThisType dot(const ThisType& rhs) const { return matmul(rhs); }

    ThisType matmul(const ThisType& rhs) const
    {
        const ThisType& lhs(*this);
        if(lhs.shape(1) != rhs.shape(0))
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
        ThisType ret(ThisType::zeros({lhs.shape(0), rhs.shape(1)}));
        ret.traverse([&](size_t i, size_t j)
            {
                for(int k = 0; k < lhs.shape(1); k++)
                    ret(i,j) += lhs(i, k) * rhs(k, j);
            });
        return ret;
    }

    std::string str() const
    {
        std::string ret;
        traverse([&](size_t i, size_t j){
            ret += (rtc::to_string((*this)(i,j), 6) + (j == shape(1) - 1 ? "\n" : " ")); });
        return ret;
    }

    ThisType& setBlock(size_t i0, size_t j0, const ThisType& mat)
    {
        mat.traverse([&](size_t i, size_t j) {(*this)(i + i0, j + j0) = mat(i, j);});
        return *this;
    }
    const MatRef operator () (const Block& s) const;
    MatRef operator () (const Block& s);

protected:
    virtual std::vector<FloatType>* dataVectorPtr() { return &data_; }
    virtual const std::vector<FloatType>* dataVectorPtr() const { return &data_; }
    virtual bool ownerMajor() const {return majorAxis();}
    virtual Shape ownerShape() const {return shape();}
    virtual Shape refOffset() const { return Shape({0,0}); }

protected:
    Shape shape_;
    std::vector<FloatType> data_;
    bool major_;
};

inline Mat operator + (FloatType lhs, const Mat& rhs) { return rhs + lhs;}
inline Mat operator - (FloatType lhs, const Mat& rhs) { return -rhs + lhs;}
inline Mat operator * (FloatType lhs, const Mat& rhs) { return rhs * lhs;}

inline size_t indexConvert2D(size_t i, size_t j, bool major, size_t shape_i, size_t shape_j)
{
    if(major == Mat::ROW) return (i * shape_j + j);
    return (j * shape_i + i);
}

inline FloatType Mat::norm(uint8_t p) const
{
    if(2 == p)
    {
        FloatType sum2(0);
        Mat mat_2((*this)*(*this));
        traverse([&](size_t i, size_t j){sum2 += mat_2(i,j);});
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

} // namespace rtc

#endif // _LINALG_MAT_H
