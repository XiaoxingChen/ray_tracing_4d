#if !defined(_LINALG_MAT_REF_H)
#define _LINALG_MAT_REF_H

#include "linalg_mat.h"


namespace rtc
{

inline std::array<size_t,2> indexBlockToMat(size_t i, size_t j, size_t offset_i, size_t offset_j, bool same_major)
{
    if(same_major) return std::array<size_t,2>{offset_i + i, offset_j + j};
    return std::array<size_t,2>{offset_i + j, offset_j + i};
}

template<typename DType>
class MatrixRef: public Matrix<DType>
{
public:
    using ConstDataPtr = const std::vector<DType>*;
    using ThisType = MatrixRef;
    using BaseType = Matrix<DType>;
    using DataPtr = typename Matrix<DType>::DataPtr;

    MatrixRef(
        const Shape& shape, const Shape& owner_shape,
        bool major, bool owner_major,
        const Shape& offset,
        DataPtr p_owner_data)
    :BaseType({0,0}, {}, major),
        owner_shape_(owner_shape), owner_major_(owner_major),
        ref_offset_(offset),
        p_owner_data_(p_owner_data)
        {
            BaseType::shape_ = shape;
        }


    virtual bool ownerMajor() const override { return owner_major_; }
    virtual Shape refOffset() const override { return ref_offset_; }
    virtual Shape ownerShape() const {return owner_shape_;}

    virtual DataPtr dataVectorPtr() override { return p_owner_data_; }
    virtual const DataPtr dataVectorPtr() const override { return p_owner_data_; }

    virtual const DType& operator () (size_t i, size_t j) const override
    {
        auto ref_ij = indexBlockToMat(i,j, ref_offset_[0], ref_offset_[1], owner_major_ == BaseType::majorAxis());
        return dataVectorPtr()->at(
            indexConvert2D(ref_ij[0], ref_ij[1], owner_major_, owner_shape_[0], owner_shape_[1]));
    }

    virtual DType& operator () (size_t i, size_t j) override { return const_cast<DType&>(static_cast<const ThisType&>(*this)(i,j)); }
    virtual const MatrixRef<DType> operator () (const Block& s) const override
    {
        return Matrix<DType>::operator () (s);
    }

    virtual MatrixRef<DType> operator () (const Block& s) override
    {
        return static_cast<const ThisType&>(*this)(s);
    }

    //
    // All overloaded operators except assignment (operator=)
    // are inherited by derived classes.
    virtual void operator = (const Matrix<DType>& rhs)
    {
        if(BaseType::shape() != rhs.shape())
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
        Matrix<DType>::traverse([&](size_t i, size_t j) { (*this)(i,j) = rhs(i,j); });
    }

protected:
    Shape owner_shape_;
    bool owner_major_;
    Shape ref_offset_;

    DataPtr p_owner_data_;
    ConstDataPtr cp_owner_data_;
};

template <typename DType>
const MatrixRef<DType> Matrix<DType>::T() const
{
    return MatrixRef<DType>(
        {shape(1), shape(0)}, ownerShape(),
        !majorAxis(), ownerMajor(),
        refOffset(),
        const_cast<typename Matrix<DType>::DataPtr>(dataVectorPtr()));
}

template <typename DType>
MatrixRef<DType> Matrix<DType>::T()
{
    return static_cast<const Matrix<DType>&>(*this).T();
}

template <typename DType>
const MatrixRef<DType> Matrix<DType>::operator () (const Block& s) const
{
    auto i01j01 = s.getBlock(*this);
    auto newOffset = refOffset();
    if(ownerMajor() == majorAxis())
    {
        newOffset[0] += i01j01[0];
        newOffset[1] += i01j01[2];
    }
    else
    {
        newOffset[0] += i01j01[2];
        newOffset[1] += i01j01[0];
    }

    return MatrixRef<DType>(
        {i01j01[1] - i01j01[0], i01j01[3] - i01j01[2]}, ownerShape(),
        majorAxis(), ownerMajor(),
        newOffset, const_cast<typename Matrix<DType>::DataPtr>(dataVectorPtr()));
}

template <typename DType>
MatrixRef<DType> Matrix<DType>::operator () (const Block& s)
{
    return static_cast<const Matrix<DType>&>(*this)(s);
}

using MatRef = MatrixRef<FloatType>;


} // namespace rtc

#endif // _LINALG_MAT_REF_H
