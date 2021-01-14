#if !defined(_LINALG_MAT_REF_H)
#define _LINALG_MAT_REF_H

#include "linalg_mat.h"


namespace rtc
{
class MatRef: public Mat
{
public:

    ~MatRef(){}
    // virtual size_t shape(uint8_t i) const override {return ref_shape_.at(i);}
    // virtual const Shape& shape() const override {return ref_shape_;}
    std::array<size_t,2> toOwnerCoordinate(size_t i, size_t j) const
    {
        if(rowMajor() == owner_row_major_)
        {
            return std::array<size_t,2>{i + ref_offset_[0], j + ref_offset_[1]};
        }
        return std::array<size_t,2>{j + ref_offset_[0], i + ref_offset_[1]};
    }

    virtual FloatType& operator () (size_t i, size_t j)
    {
        auto ref_ij = toOwnerCoordinate(i,j);
        return (p_owner_data_)->at(
            indexConvert2D(ref_ij[0], ref_ij[1], owner_row_major_, owner_shape_[0], owner_shape_[1]));
    }
    virtual const FloatType& operator () (size_t i, size_t j) const
    {
        auto ref_ij = toOwnerCoordinate(i,j);
        return (p_owner_data_)->at(
            indexConvert2D(ref_ij[0], ref_ij[1], owner_row_major_, owner_shape_[0], owner_shape_[1]));
    }

// protected:
    using DataPtr = std::vector<FloatType>*;
    MatRef(const Shape& shape, const Mat& owner, const Shape& offset, bool col_major, DataPtr p_owner_data)
        :Mat(shape, {}, col_major),
        owner_shape_(owner.shape()),
        owner_row_major_(owner.rowMajor()),
        p_owner_data_(p_owner_data),
        ref_offset_(offset) {}

protected:

    Shape owner_shape_;
    bool owner_row_major_;
    DataPtr p_owner_data_;
    Shape ref_offset_;


};

inline MatRef Mat::T()
{
    return MatRef({shape(1), shape(0)}, *this, {0,0}, rowMajor(), &this->data_);
}
} // namespace rtc

#endif // _LINALG_MAT_REF_H
