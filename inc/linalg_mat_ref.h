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

class MatRef: public Mat
{
public:
    using ConstDataPtr = const std::vector<FloatType>*;
    using DataPtr = std::vector<FloatType>*;

    MatRef(
        const Shape& shape, const Shape& owner_shape,
        bool major, bool owner_major,
        const Shape& offset,
        DataPtr p_owner_data)
    :Mat({0,0}, {}, major),
        owner_shape_(owner_shape), owner_major_(owner_major),
        ref_offset_(offset),
        p_owner_data_(p_owner_data)
        {
            shape_ = shape;
        }

    using ThisType = MatRef;

    virtual bool ownerMajor() const override { return owner_major_; }
    virtual Shape refOffset() const override { return ref_offset_; }
    virtual Shape ownerShape() const {return owner_shape_;}

    virtual std::vector<FloatType>* dataVectorPtr() override { return p_owner_data_; }
    virtual const std::vector<FloatType>* dataVectorPtr() const override { return p_owner_data_; }

    virtual const FloatType& operator () (size_t i, size_t j) const
    {
        auto ref_ij = indexBlockToMat(i,j, ref_offset_[0], ref_offset_[1], owner_major_ == majorAxis());
        return dataVectorPtr()->at(
            indexConvert2D(ref_ij[0], ref_ij[1], owner_major_, owner_shape_[0], owner_shape_[1]));
    }

    virtual FloatType& operator () (size_t i, size_t j)
    {
        static FloatType dummy(666);
        if(!dataVectorPtr()) return dummy;
        return const_cast<FloatType&>(
            static_cast<const ThisType&>(*this)(i,j));
    }
    virtual void operator = (const Mat& rhs)
    {
        traverse([&](size_t i, size_t j)
            {
                (*this)(i,j) = rhs(i,j);
            });
    }

protected:
    Shape owner_shape_;
    bool owner_major_;
    Shape ref_offset_;

    DataPtr p_owner_data_;
    ConstDataPtr cp_owner_data_;
};

inline const MatRef Mat::T() const
{
    return MatRef(
        {shape(1), shape(0)}, ownerShape(),
        !majorAxis(), ownerMajor(),
        refOffset(),
        const_cast<MatRef::DataPtr>(dataVectorPtr()));
}

inline MatRef Mat::T()
{
    return static_cast<const Mat&>(*this).T();
}

#if 1
inline const MatRef Mat::operator () (const Block& s) const
{
    auto i01j01 = s.getBlock(*this);
    auto newOffset = refOffset();
    newOffset[0] += i01j01[0];
    newOffset[1] += i01j01[2];
    return MatRef(
        {i01j01[1] - i01j01[0], i01j01[3] - i01j01[2]}, ownerShape(),
        majorAxis(), ownerMajor(),
        newOffset, const_cast<MatRef::DataPtr>(dataVectorPtr()));
}

inline MatRef Mat::operator () (const Block& s)
{
    return static_cast<const Mat&>(*this)(s);
}
#endif

#include "linalg_vec.h"


inline Vec::Vec(const Mat& mat)
    : Mat(mat.shape(1) == 1 ? mat : mat.T())
{
    if(mat.shape(0) != 1 && mat.shape(1) != 1)
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
}

} // namespace rtc

#endif // _LINALG_MAT_REF_H
