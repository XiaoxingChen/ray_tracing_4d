#if !defined(_LINALG_H_)
#define _LINALG_H_

#include <vector>
#include <array>
#include <string>
#include <assert.h>
#include <math.h>
#include "base_type.h"

namespace rtc
{
using FloatType = float_t;

class Mat
{
public:
    using size_t = uint32_t;
    using Shape = std::array<size_t, 2>;

    // const Shape& shape() const {return shape_;}
    const Shape shape;
    bool square() const {return shape.at(0) == shape.at(1);}
    bool rowMajor() const {return row_major_;}

    Mat(const Shape& shape_, const std::vector<FloatType>& data={})
        :shape(shape_), row_major_(true)
    {
        if(data.size() == 0)
        {
            data_.resize(shape.at(0) * shape.at(1), 0.);
            return;
        }
        if(shape.at(0) * shape.at(1) != data.size())
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
            return data_.at(i * shape.at(1) + j);
        else
            return data_.at(j * shape.at(0) + i);
    }

    const FloatType& operator () (size_t i, size_t j) const
    {
        if(row_major_)
            return data_.at(i * shape.at(1) + j);
        else
            return data_.at(j * shape.at(0) + i);
    }

    Mat& operator *= (FloatType scalar)  { for(auto & v: data_) v *= scalar; return *this;}
    Mat operator * (FloatType scalar) const { return Mat(*this) *= scalar; }

    Mat& operator -= (const Mat& rhs )
    {
        for(int i = 0; i < data_.size(); i++)
            data_.at(i) -= rhs.data_.at(i);
        return *this;
    }
    Mat operator - (const Mat& rhs) const { return Mat(*this) -= rhs; }

    FloatType norm(int8_t p = 'f')
    {
        if('f' == p)
        {
            FloatType sum2(0);
            for(auto & v : data_) sum2 += v*v;
            return sqrt(sum2);
        }

        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
        return 0.;
    }

    Mat T() const
    {
        Mat ret({shape.at(1), shape.at(0)}, data_);
        ret.row_major_ = !row_major_;
        return ret;
    }

    Mat dot(const Mat& rhs) const
    {
        const Mat& lhs(*this);
        if(lhs.shape.at(1) != rhs.shape.at(0))
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
        Mat ret({lhs.shape.at(0), rhs.shape.at(1)});
        for(int i = 0; i < ret.shape.at(0); i++)
        {
            for(int j = 0; j < ret.shape.at(1); j++)
            {
                ret(i,j) = 0;
                for(int k = 0; k < lhs.shape.at(1); k++)
                {
                    ret(i,j) += lhs(i, k) * rhs(k, j);
                }
            }
        }
        return ret;
    }

    std::string Str() const
    {
        std::string ret;
        for(int i = 0; i < shape.at(0); i++)
        {
            for(int j = 0; j < shape.at(1); j++)
            {
                ret += (std::to_string((*this) (i,j)) + " ");
            }
            ret += "\n";
        }
        return ret;
    }

private:
    // Shape shape_;
    std::vector<FloatType> data_;
    bool row_major_;
};

} // namespace rtc


#endif // _LINALG_H_
