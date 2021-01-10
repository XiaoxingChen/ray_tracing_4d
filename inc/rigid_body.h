#if !defined(__RIGID_BODY_H__)
#define __RIGID_BODY_H__
#include "ray.h"
#include "rotation.h"

#include <memory>
#include <string>
#include "primitive_geometry.h"
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
        virtual Vec center() const = 0;
        virtual std::string str() const {return "";};

        static std::shared_ptr<RigidBody> choose(Types type, size_t dimension, const std::vector<FloatType>& args);
        static std::shared_ptr<RigidBody> choose(Types type, VecIn position, const Rotation& orientation, const std::vector<FloatType>& args);
        static std::shared_ptr<RigidBody> createPrimitiveMesh(VecIn position, const Rotation& orientation, const Mat& primitives, const std::vector<std::vector<size_t>>& indices);
};

using RigidBodyPtr = std::shared_ptr<RigidBody>;

inline RigidBody::HitRecordPtr hitPrimitivePolygon(
    const Ray& ray, std::shared_ptr<Mat> p_vertex_buffer, const std::vector<size_t>& indices)
{
    size_t dim(indices.size());
    if(dim != ray.origin().size())
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    Mat mat_a({dim, dim});
    for(size_t i = 0; i < dim; i++)
    {
        mat_a.set(Col(i), (*p_vertex_buffer)(Col(indices.at(i))));
    }
    Vec result = intersectEquation(mat_a, ray);
    if(!validIntersect(result)) return nullptr;

    FloatType intersection_t = result(0);

    if(!ray.valid(intersection_t)) return nullptr;

    Mat norm_complement(mat_a(Block({}, {1, dim})) - mat_a(Block({}, {0, dim-1})));

    Vec norm = orthogonalComplement(norm_complement);

    auto ret = std::make_shared<RigidBody::HitRecord>(
        intersection_t, ray(intersection_t), norm);

    return ret;
}

} // namespace rtc

#endif // __RIGID_BODY_H__
