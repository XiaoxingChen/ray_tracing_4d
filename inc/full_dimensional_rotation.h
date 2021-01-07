#if !defined(_FULL_DIMENSIONAL_ROTATION_H_)
#define _FULL_DIMENSIONAL_ROTATION_H_

#include "linalg.h"
#include "rotation_base.h"
#include <iostream>


namespace rtc
{

class FullDimensionalRotation
{
public:
    using ThisType = FullDimensionalRotation;

    FullDimensionalRotation(const Mat& R): matrix_(R)
    {
        if(!matrix_.square())
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

    ThisType operator*(const ThisType& rhs) const { return ThisType(matrix_.matmul(rhs.matrix_)); }

    Mat apply(const Mat& vector) const { return matrix_.matmul(vector); }

    Mat asMatrix() const { return matrix_; }

    ThisType inv() const { return ThisType(matrix_.T()); }

    size_t dim() const { return matrix_.shape(0); }

    static ThisType fromMatrix(const Mat& R) { return ThisType(R); }
    static ThisType fromAngle(FloatType angle) { return ThisType(rtc::rodrigues2D(angle)); }
    static ThisType fromAxisAngle(UnitVecIn axis, FloatType angle) { return ThisType(rtc::rodrigues3D(axis, angle)); }
    static ThisType fromPlaneAngle(UnitVecIn u, UnitVecIn v, FloatType angle)
    {
        auto bivec = planeAngleToBivector(u, v, angle);
        return ThisType(bivectorToRotationMatrix(bivec[0], bivec[1]));
    }

    static ThisType Identity(size_t dim) { return ThisType(Mat::Identity(dim)); }



private:
    Mat matrix_;
};

} // namespace rtc


#endif // _FULL_DIMENSIONAL_ROTATION_H_
