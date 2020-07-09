#if !defined(_ROTATION_H_)
#define _ROTATION_H_
#include "vec3.h"
#include <math.h>

namespace rtc
{
class Mat3;
class Rotation
{
public:
    Rotation(/* args */) {}
    ~Rotation() {}
    using ThisType = Rotation;
    ThisType operator*(const ThisType& rhs) const
    {

    }

    V3 apply(const V3& vector);
    V3 apply(const V3& vector);

private:

    Rotation(const Mat3& R);
    Mat3 rodrigues() const;


    UnitVector3 axis_;
    float_t angle_;
};
} // namespace rtc



#endif // _ROTATION_H_
