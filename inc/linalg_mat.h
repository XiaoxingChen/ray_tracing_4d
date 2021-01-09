#if !defined(_LINALG_MAT_H)
#define _LINALG_MAT_H

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
class Block;
class Mat
{
public:
    using Shape = std::array<size_t, 2>;

    size_t shape(uint8_t i) const {return shape_.at(i);}
    const Shape& shape() const {return shape_;}
    // const Shape shape;
    bool square() const {return shape(0) == shape(1);}
    bool rowMajor() const {return row_major_;}

    Mat(const Shape& _shape, const std::vector<FloatType>& data={})
        :shape_(_shape), data_(_shape[0] * _shape[1], 0), row_major_(true)
    {
        if(data.size() == 0)
        {
            return;
        }
        if(shape(0) * shape(1) != data.size())
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
        data_ = data;
    }

    // Mat(Mat && mat): shape(mat.shape), data_(std::move(mat.data_)), row_major_(mat.row_major_){}

    static Mat zeros(const Shape& _shape) { return Mat(_shape); }

    static Mat ones(const Shape& _shape) { return Mat(_shape) + 1; }

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

    Mat operator -() const        { return Mat(*this) *= -1;}

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

    Mat block(const std::vector<size_t>& i, const std::vector<size_t>& j) const
    {
        std::array<size_t, 2> j_in({0, shape(1)});
        std::array<size_t, 2> i_in({0, shape(0)});

        for(size_t idx = 0; idx < j.size(); idx++) j_in.at(idx) = j.at(idx);
        for(size_t idx = 0; idx < i.size(); idx++) i_in.at(idx) = i.at(idx);

        return block_(i_in, j_in);
    }

    Mat& setBlock(size_t i0, size_t j0, const Mat& mat)
    {
        for(size_t i = 0; i < mat.shape(0); i++)
        {
            for(size_t j = 0; j < mat.shape(1); j++)
            {
                (*this)(i + i0, j + j0) = mat(i, j);
            }
        }
        return *this;
    }

    Mat operator () (const Block& s) const;
    Mat& set(const Block& s, const Mat& rhs);

protected:
    Mat block_(
        const std::array<size_t, 2>& row,
        const std::array<size_t, 2>& col) const
    {
        Mat ret({row[1] - row[0], col[1] - col[0]});
        for(size_t i = 0; i < ret.shape(0); i++)
        {
            for(size_t j = 0; j < ret.shape(1); j++)
            {
                ret(i,j) = (*this)(i + row[0], j + col[0]);
            }
        }
        return ret;
    }

protected:
    Shape shape_;
    std::vector<FloatType> data_;
    bool row_major_;
};

inline Mat operator + (FloatType lhs, const Mat& rhs) { return rhs + lhs;}
inline Mat operator - (FloatType lhs, const Mat& rhs) { return -rhs + lhs;}
inline Mat operator * (FloatType lhs, const Mat& rhs) { return rhs * lhs;}


} // namespace rtc

#endif // _LINALG_MAT_H
