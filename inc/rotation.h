#if !defined(_ROTATION_H_)
#define _ROTATION_H_
#include "linalg.h"
#include <math.h>
#include <string>
namespace rtc
{
class Rotation
{
public:
    Rotation(FloatType angle)
    :plane_(Mat::Identity(2)), angle_(angle){ }

    Rotation(const Vec& axis, FloatType angle)
    :plane_(orthogonalComplement(axis)), angle_(angle) { checkDimension(); }

    Rotation(const Mat& plane, FloatType angle)
    :plane_(plane), angle_(angle){ checkDimension(); }

    Rotation()
    :plane_(Mat({2, 3},{1,0,0, 0,1,0}).T()), angle_(0){}

    ~Rotation() {}
    using ThisType = Rotation;
    ThisType operator*(const ThisType& rhs) const;

    Mat apply(const Mat& vector) const;

    static Rotation fromMatrix(const Mat& R);
    Mat asMatrix() const;

    Rotation inv() const { return Rotation(plane_, -angle_); }

    std::string str() const
    {
        return std::string("plane: \n") + plane_.str() + "angle: " + std::to_string(angle_);
    }

    void checkDimension() const
    {
        if(plane_.shape(1) != 2)
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

    size_t dim() const { return plane_.shape(0); }

    static Rotation Identity(size_t dim)
    {
        Mat plane({dim, 2});
        plane(0,0) = 1;
        plane(1,1) = 1;
        return Rotation(plane, 0.);
    }

private:
    Mat rodrigues() const;
    Mat rodrigues2D() const;
    Mat rodrigues3D() const;

    Mat plane_;
    FloatType angle_;
};
} // namespace rtc



#endif // _ROTATION_H_
