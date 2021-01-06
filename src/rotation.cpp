#include "rotation.h"
// #include "vec3.h"
#include "linalg.h"
#include <cmath>
#include "rotation_base.h"


namespace rtc
{
    using M3 = Mat;

    LowDimensionalRotation LowDimensionalRotation::fromMatrix(const Mat& R)
    {
        UnitVec r(3);
        FloatType angle;
        matrixToAxisAngle3D(R, r, angle);
        Mat plane = orthogonalComplement(r);
        return LowDimensionalRotation(plane, angle);
    }

    Mat LowDimensionalRotation::asMatrix() const
    {
        return rodrigues();
    }

    Mat LowDimensionalRotation::rodrigues() const
    {
        if(2 == dim()) return rodrigues2D();
        return rodrigues3D();
    }

    Mat LowDimensionalRotation::rodrigues3D() const
    {
        Vec axis(orthogonalComplement(plane_));
        return rtc::rodrigues3D(axis, angle_);
    }

    Mat LowDimensionalRotation::rodrigues2D() const
    {
        return rtc::rodrigues2D(angle_);
    }

    LowDimensionalRotation LowDimensionalRotation::operator*(const ThisType& rhs) const
    {
        return fromMatrix(rodrigues().dot(rhs.rodrigues()));
    }

    Mat LowDimensionalRotation::apply(const Mat& vectors) const
    {
        return rodrigues().matmul(vectors);
        // return rodrigues();
    }
} // namespace rtc
