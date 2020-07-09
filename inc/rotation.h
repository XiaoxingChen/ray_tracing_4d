#if !defined(_ROTATION_H_)
#define _ROTATION_H_
#include "vec3.h"
#include <math.h>

namespace rtc
{
class Rotation
{

public:
    Rotation(/* args */) {}
    ~Rotation() {}
    using ThisType = Rotation;
    ThisType operator*(const ThisType& rhs) const
    {

    }
private:
    using M3 = std::array<V3, 3>;
    Rotation(const M3& R);
    M3 rodrigues() const;
    

    UnitVector3 axis_;
    float_t angle_;
};
} // namespace rtc



#endif // _ROTATION_H_
