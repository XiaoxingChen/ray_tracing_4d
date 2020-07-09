#if !defined(__RIGID_BODY_H__)
#define __RIGID_BODY_H__
#include "ray.h"
#include <memory>
#include <string>
// #include "hittable.h"

namespace rtc
{

// class HitRecordPtr;
class RigidBody
{
    public:
        struct HitRecord 
        {
            HitRecord(float_t t_, const Vector3& p_, const Vector3& n_)
            :t(t_), p(p_), n(n_){}
            float_t t; //hit t
            Vector3 p; //hit point
            UnitVector3 n; //normal vector
        };
        RigidBody(){}
        
        enum Types
        {
            SPHERE,
            CUBE,
            CYLLINDER,
            ELLIPSOID,
            CUBOID
        };
        using HitRecordPtr = std::shared_ptr<HitRecord>;
        virtual HitRecordPtr hit(const Ray& ray) const = 0;
        virtual std::string str() const {};

        static std::shared_ptr<RigidBody> choose(Types type, const Vector3& center=V3(), const Vector3& size=V3{1,1,1});
};

using RigidBodyPtr = std::shared_ptr<RigidBody>;

} // namespace rtc

#endif // __RIGID_BODY_H__
