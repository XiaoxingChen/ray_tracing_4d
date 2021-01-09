#if !defined(__RIGID_BODY_H__)
#define __RIGID_BODY_H__
#include "ray.h"
#include "rotation.h"

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
            HitRecord(float_t t_, const std::vector<FloatType>& p_, const std::vector<FloatType>& n_)
            :t(t_), p(p_), n(n_){}
            HitRecord(float_t t_, const Vec& p_, const Vec& n_)
            :t(t_), p(p_), n(n_){}
            HitRecord(size_t dim=3) :t(0), p(dim), n(dim){}
            float_t t; //hit t
            Vec p; //hit point
            UnitVec n; //normal vector
        };
        RigidBody(){}

        enum Types
        {
            // args:
            // radius: argv[0]
            SPHERE,

            // args:
            // radius: argv[0:dim],
            RECTANGLE,
            CUBE,

            // args:
            // radius: argv[0],
            // height: argv[1],
            CYLINDER,
            ELLIPSOID
        };
        using HitRecordPtr = std::shared_ptr<HitRecord>;
        virtual HitRecordPtr hit(const Ray& ray) const = 0;
        virtual std::string str() const {return "";};

        static std::shared_ptr<RigidBody> choose(Types type, size_t dimension, const std::vector<FloatType>& args);
        static std::shared_ptr<RigidBody> choose(Types type, VecIn position, const Rotation& orientation, const std::vector<FloatType>& args);
        static std::shared_ptr<RigidBody> createPrimitiveMesh(VecIn position, const Rotation& orientation, const Mat& primitives, const std::vector<std::vector<size_t>>& indices);
};

using RigidBodyPtr = std::shared_ptr<RigidBody>;

} // namespace rtc

#endif // __RIGID_BODY_H__
