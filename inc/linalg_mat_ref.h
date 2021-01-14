#if !defined(_LINALG_MAT_REF_H)
#define _LINALG_MAT_REF_H

#include "linalg_mat.h"


namespace rtc
{

inline std::array<size_t,2> indexBlockToMat(size_t i, size_t j, size_t offset_i, size_t offset_j, bool same_major)
{
    if(same_major) return std::array<size_t,2>{i + offset_i, j + offset_j};
    return std::array<size_t,2>{j + offset_i, i + offset_j};
}

inline const FloatType& accessOwnerData(
    size_t i, size_t j,
    bool owner_major, bool ref_major,
    const Mat::Shape& offset,
    const Mat::Shape& owner_shape,
    const std::vector<FloatType>* p_data)
{
    auto ref_ij = indexBlockToMat(i,j, offset[0], offset[1], owner_major == ref_major);
    return p_data->at(
        indexConvert2D(ref_ij[0], ref_ij[1], owner_major, owner_shape[0], owner_shape[1]));
}

inline FloatType& accessOwnerData(
    size_t i, size_t j,
    bool owner_major, bool ref_major,
    const Mat::Shape& offset,
    const Mat::Shape& owner_shape,
    std::vector<FloatType>* p_data)
{
    auto ref_ij = indexBlockToMat(i,j, offset[0], offset[1], owner_major == ref_major);
    return p_data->at(
        indexConvert2D(ref_ij[0], ref_ij[1], owner_major, owner_shape[0], owner_shape[1]));
}

class MatRefBase: public Mat
{
public:
    MatRefBase(const Shape& shape, const Shape& owner_shape, bool major, bool owner_major, const Shape& offset)
        :Mat({0,0}, {}, major),
        owner_shape_(owner_shape),
        owner_major_(owner_major),
        ref_offset_(offset) { shape_ = shape; }

protected:

    Shape owner_shape_;
    bool owner_major_;
    Shape ref_offset_;
};

class ConstMatRef: public MatRefBase
{
public:
    using ConstDataPtr = const std::vector<FloatType>*;
    ConstMatRef(
        const Shape& shape, const Shape& owner_shape, bool major, bool owner_major, const Shape& offset, ConstDataPtr p_data)
        :MatRefBase(shape, owner_shape, major, owner_major, offset),
        p_owner_data_(p_data) {}

    virtual const FloatType& operator () (size_t i, size_t j) const
    {
        return accessOwnerData(i, j, owner_major_, majorAxis(), ref_offset_, owner_shape_, p_owner_data_);
    }

    operator Mat () const { return Mat(shape(), *p_owner_data_); }

protected:
    ConstDataPtr p_owner_data_;
};

class MatRef: public MatRefBase
{
public:
    using DataPtr = std::vector<FloatType>*;
    MatRef(
        const Shape& shape, const Shape& owner_shape,
        bool major, bool owner_major,
        const Shape& offset, DataPtr p_data)
        :MatRefBase(shape, owner_shape, major, owner_major, offset),
        p_owner_data_(p_data) {}

    virtual void operator = (const Mat& rhs) override { Mat::operator=(rhs); }

    virtual FloatType& operator () (size_t i, size_t j)
    {
        return accessOwnerData(i, j, owner_major_, majorAxis(), ref_offset_, owner_shape_, p_owner_data_);
    }
    virtual const FloatType& operator () (size_t i, size_t j) const
    {
        return accessOwnerData(i, j, owner_major_, majorAxis(), ref_offset_, owner_shape_, p_owner_data_);
    }

    operator ConstMatRef () const
    {
        return ConstMatRef(shape(), owner_shape_, majorAxis(), owner_major_, {0,0}, &this->data_);
    }

    operator Mat () const { return Mat(shape(), *p_owner_data_); }
protected:
    DataPtr p_owner_data_;
};

// inline MatRef Mat::T()
// {
//     return MatRef({shape(1), shape(0)}, *this, {0,0}, rowMajor(), &this->data_);
// }
inline ConstMatRef Mat::T() const
{
    return ConstMatRef(
        {shape(1), shape(0)}, shape(),
        !majorAxis(), majorAxis(),
        {0,0}, &this->data_);
}
inline MatRef Mat::T()
{
    return MatRef(
        {shape(1), shape(0)}, shape(),
        !majorAxis(), majorAxis(),
        {0,0}, &this->data_);
}
} // namespace rtc

#endif // _LINALG_MAT_REF_H
