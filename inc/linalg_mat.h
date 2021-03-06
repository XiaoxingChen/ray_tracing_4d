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

inline std::string to_string(const std::array<size_t, 2>& s)
{
    return std::to_string(s[0]) + " "  + std::to_string(s[1]);
}
// class Vec;
class Block;
template<typename DType> class MatrixRef;
// using MatRef = MatrixRef<FloatType>;

using Shape = std::array<size_t, 2>;
template<typename DType>
class Matrix
{
public:

    static const bool ROW = 0;
    static const bool COL = 1;
    using ThisType = Matrix<DType>;
    using DataPtr = std::vector<DType>*;

    //
    // interfaces
    virtual size_t shape(uint8_t i) const {return shape_.at(i);}
    virtual const Shape& shape() const {return shape_;}
    bool square() const {return shape(0) == shape(1);}
    bool majorAxis() const {return major_;}
    bool& majorAxis() {return major_;}
    virtual ThisType& owner() { return *this; }
    virtual const ThisType& owner() const { return *this; }
    virtual const Shape& absOffset() const { static Shape zero({0,0});  return zero; }

    //
    // constructors
    Matrix():shape_({0,0}), data_(), major_(ROW) {}

    Matrix(const Shape& _shape, const std::vector<DType>& data={}, bool major=ROW)
        :shape_(_shape), data_(_shape[0] * _shape[1], DType()), major_(major)
    {
        if(data.size() == 0)  return;
        if(shape(0) * shape(1) != data.size())
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
        data_ = data;
    }

    Matrix(const ThisType& rhs)
        :shape_(rhs.shape()), data_(rhs.shape(0) * rhs.shape(1)), major_(rhs.majorAxis())
    {
        (*this) = rhs;
    }

    Matrix(ThisType&& rhs)
        :shape_({0,0}), data_(0), major_(ROW)
    {
        (*this) = std::move(rhs);
    }

    virtual void operator = (ThisType&& rhs)
    {
        if(&rhs.owner() == &rhs) // not MatrixRef<DType>
        {
            shape_.swap(rhs.shape_);
            data_.swap(rhs.data_);
            major_ = rhs.major_;
            return;
        }
        if(rhs.owner().shape(0) * rhs.owner().shape(1) == rhs.shape(0) * rhs.shape(1)) // transposed MatrixRef<DType>
        {
            shape_.swap(rhs.shape_);
            data_.swap(rhs.owner().data_);
            major_ = rhs.major_;
            return ;
        }

        (*this) = rhs;
    }

    virtual void operator = (const ThisType& rhs)
    {
        if(this->shape() == Shape{0,0})
        {
            shape_ = rhs.shape();
            data_.resize(shape(0) * shape(1));
            major_ = rhs.majorAxis();
        }
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
    virtual const DType& operator () (size_t i, size_t j) const
    {
        return data_.at(
            indexConvert2D(i,j,majorAxis(), shape(0), shape(1)));
    }

    virtual DType& operator () (size_t i, size_t j) { return const_cast<DType&>(static_cast<const ThisType&>(*this)(i,j)); }

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
    ThisType& operator *= (DType scalar)  { traverse([&](size_t i, size_t j){(*this)(i,j) *= scalar;}); return *this;}
    ThisType& operator += (DType scalar)  { traverse([&](size_t i, size_t j){(*this)(i,j) += scalar;}); return *this;}
    ThisType& operator -= (DType scalar)  { traverse([&](size_t i, size_t j){(*this)(i,j) -= scalar;}); return *this;}

    ThisType operator * (DType scalar) const { return ThisType(*this) *= scalar; }
    ThisType operator + (DType scalar) const { return ThisType(*this) += scalar; }
    ThisType operator - (DType scalar) const { return ThisType(*this) -= scalar; }

    ThisType& operator *= (const ThisType& rhs)  { traverse([&](size_t i, size_t j){(*this)(i,j) *= rhs(i,j);}); return *this;}
    ThisType& operator += (const ThisType& rhs)  { traverse([&](size_t i, size_t j){(*this)(i,j) += rhs(i,j);}); return *this;}
    ThisType& operator -= (const ThisType& rhs)  { traverse([&](size_t i, size_t j){(*this)(i,j) -= rhs(i,j);}); return *this;}

    ThisType operator * (const ThisType& rhs) const { return ThisType(*this) *= rhs; }
    ThisType operator + (const ThisType& rhs) const { return ThisType(*this) += rhs; }
    ThisType operator - (const ThisType& rhs) const { return ThisType(*this) -= rhs; }

    ThisType operator -() const        { return ThisType(*this) *= -1;}

    DType det() const;
    ThisType inv() const;
    DType trace() const
    {
        if(!square()) throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
        DType ret(0);
        for(size_t i = 0; i < shape(0); i++) ret += (*this)(i,i);
        return ret;
    }

    DType norm(uint8_t p = 2) const;
    DType norm(const std::string& p) const
    {
        if(p == std::string("F")) return norm(2);
        if(p == std::string("inf")) return norm(255);
        return 0;
    }

    const ThisType& normalize(uint8_t p = 2)
    {
        DType n = norm(p);
        if(n < eps()*eps()) { return *this; }
        return ((*this) *= (1./n));
    }

    ThisType normalized() const { return ThisType(*this).normalize(); }

    virtual const MatrixRef<DType> T() const;
    virtual MatrixRef<DType> T();

    ThisType dot(const ThisType& rhs) const { return matmul(rhs); }

    template<typename RhsDType>
    auto matmul(const Matrix<RhsDType>& rhs) const -> Matrix<decltype(DType() * RhsDType())>
    {
        using ReturnType = Matrix<decltype(DType() * RhsDType())>;
        const ThisType& lhs(*this);
        if(lhs.shape(1) != rhs.shape(0))
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
        ReturnType ret({lhs.shape(0), rhs.shape(1)});
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
    virtual const MatrixRef<DType> operator () (const Block& s) const;
    virtual MatrixRef<DType> operator () (const Block& s);
    ThisType pow(size_t n) const
    {
        ThisType ret = ThisType::ones(shape());
        for(size_t i = 0; i < n; i++) ret *= (*this);
        return ret;
    }


protected:
    Shape shape_;
    std::vector<DType> data_;
    bool major_;
};

using Mat = Matrix<FloatType>;

template<typename LType, typename DType> Mat operator + (LType lhs, const Mat& rhs) { return rhs + lhs;}
template<typename LType, typename DType> Mat operator - (LType lhs, const Mat& rhs) { return -rhs + lhs;}
template<typename LType, typename DType> Matrix<DType> operator * (LType lhs, const Matrix<DType>& rhs) { return rhs * lhs;}

inline size_t indexConvert2D(size_t i, size_t j, bool major, size_t shape_i, size_t shape_j)
{
    if(major == Mat::ROW) return (i * shape_j + j);
    return (j * shape_i + i);
}

inline void updateOffset(
    Shape& abs_offset, const Shape& inc_offset, bool same_major)
{
    if(same_major)
    {
        abs_offset[0] += inc_offset[0];
        abs_offset[1] += inc_offset[1];
    }else
    {
        abs_offset[0] += inc_offset[1];
        abs_offset[1] += inc_offset[0];
    }
}

template <typename DType>
DType Matrix<DType>::norm(uint8_t p) const
{
    if(2 == p)
    {
        DType sum2(0);
        Mat mat_2((*this)*(*this));
        traverse([&](size_t i, size_t j){sum2 += mat_2(i,j);});
        return sqrt(sum2);
    }
    if(1 == p)
    {
        DType max_abs_sum(0);
        for(int j = 0; j < shape(1); j++)
        {
            DType abs_sum(0);
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
