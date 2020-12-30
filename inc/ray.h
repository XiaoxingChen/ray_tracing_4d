#ifndef __RAY_H__
#define __RAY_H__
#include "vec3.h"
#include "base_type.h"

namespace rtc{
class Ray
{
    public:
    Ray():direction_(V3{1,0,0}){}
    Ray(V3in origin, V3in direction, float_t t_min=0, float_t t_max=10000):
    origin_(origin), direction_(direction), t_min_(t_min), t_max_(t_max){}

    V3in origin() const       { return origin_; }
    // V3& origin()       { return origin_; }

    const UnitVector3& direction() const    { return direction_; }
    void setDirection(V3in direction)    { direction_ = direction; }

    V3 operator() (float_t t) const { return origin() + direction() * t; }
    bool valid(float_t t) const { return t < t_max_ && t > t_min_; }
    float_t tMax() const { return t_max_; }
    float_t tMin() const { return t_min_; }

    private:
    V3 origin_;
    UnitVector3 direction_;
    float_t t_min_;
    float_t t_max_;
};

// using RayPtr = std::shared_ptr<Ray>;
}//ray tracing
inline std::ostream& operator<<(std::ostream &os, const rtc::Ray& ray)
{
    os << "[" << ray.origin() << "," << ray.direction() << "]";
    return os;
}
#endif