#if !defined(_FULL_DIMENSIONAL_ROTATION_H_)
#define _FULL_DIMENSIONAL_ROTATION_H_

#include "linalg.h"
#include "rotation_base.h"
#include <iostream>


namespace rtc
{

template<typename DType>
class FullDimensionalRotation
{
public:
    using ThisType = FullDimensionalRotation;

    FullDimensionalRotation(const Matrix<DType>& R): matrix_(R)
    {
        if(!matrix_.square())
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

    FullDimensionalRotation(size_t dim): matrix_(Matrix<DType>::Identity(dim))
    {
    }

    FullDimensionalRotation(): matrix_(){}

    ThisType operator*(const ThisType& rhs) const { return ThisType(matrix_.matmul(rhs.matrix_)); }

    Matrix<DType> apply(const Matrix<DType>& vector) const { return matrix_.matmul(vector); }

    Matrix<DType> asMatrix() const { return matrix_; }

    ThisType inv() const { return ThisType(matrix_.T()); }

    size_t dim() const { return matrix_.shape(0); }

    static ThisType fromMatrix(const Matrix<DType>& R) { return ThisType(R); }
    static ThisType fromAngle(DType angle) { return ThisType(rtc::rodrigues2D(angle)); }
    static ThisType fromAxisAngle(const Vector<DType>& axis, DType angle) { return ThisType(rtc::rodrigues3D(axis, angle)); }
    static ThisType fromPlaneAngle(UnitVecIn u, UnitVecIn v, DType angle)
    {
        // todo remove UnitVecIn
        auto bivec = planeAngleToBivector(u, v, angle);
        return ThisType(bivectorToRotationMatrix(bivec[0], bivec[1]));
    }

    static ThisType Identity(size_t dim) { return ThisType(Matrix<DType>::Identity(dim)); }

    DType norm() const { return 0. ; }



private:
    Matrix<DType> matrix_;
};

} // namespace rtc


#endif // _FULL_DIMENSIONAL_ROTATION_H_
