#if !defined(__RIGID_BODY_H__)
#define __RIGID_BODY_H__
#include "ray.h"
#include "rotation.h"
#include "rigid_transform.h"

#include <memory>
#include <string>
#include "primitive_geometry.h"

namespace rtc
{

// class HitRecordPtr;
class AxisAlignedBoundingBox;
using AABB = AxisAlignedBoundingBox;

class RigidBody;
using RigidBodyPtr = std::shared_ptr<RigidBody>;
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
            size_t prim_idx;
            Vec prim_coord_hit_p;
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
        virtual void multiHit(const Ray& ray, std::vector<HitRecordPtr>& records) const
        {
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
        };
        // virtual Vec center() const = 0;
        virtual std::string str() const {return "";};
        virtual AABB aabb() const = 0;
        virtual size_t dim() const  = 0;
        virtual RigidTrans pose() const {
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
            return RigidTrans::Identity(dim());
        }

        static RigidBodyPtr choose(Types type, size_t dimension, const std::vector<FloatType>& args);
        static RigidBodyPtr choose(Types type, VecIn position, const Rotation& orientation, const std::vector<FloatType>& args);
        static RigidBodyPtr createPrimitiveMesh(const Vec& position, const Rotation& orientation, std::shared_ptr<Mat>& vertices, std::shared_ptr<Matrix<size_t>>& indices);
        static RigidBodyPtr createPolygonPrimitive(std::shared_ptr<Mat> vertex_buffer, std::shared_ptr<Matrix<size_t>>& indices, size_t prim_idx=0);
        static RigidBodyPtr createPrism(const Vec& p, const Rotation& r, FloatType h, std::shared_ptr<Mat>& vertex_buffer, std::shared_ptr<Matrix<size_t>>& vertex_index_buffer);
};

inline RigidBody::HitRecordPtr hitPrimitivePolygon(
    const Ray& ray, std::shared_ptr<Mat> p_vertex_buffer, const std::vector<size_t>& indices)
{
    size_t dim(indices.size());
    if(dim != ray.origin().size())
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    Mat mat_a({dim, dim});
    for(size_t i = 0; i < dim; i++)
    {
        mat_a(Col(i)) = (*p_vertex_buffer)(Col(indices.at(i)));
    }
    Vec result = intersectEquation(mat_a, ray);
    if(!validIntersect(result)) return nullptr;

    FloatType intersection_t = result(0);

    if(!ray.valid(intersection_t)) return nullptr;


    Vec norm = primitiveNorm(mat_a, ray);

    auto ret = std::make_shared<RigidBody::HitRecord>(
        intersection_t, ray(intersection_t), norm);

    return ret;
}

template<typename DType>
Matrix<DType> rigidBodyTransform(const Vector<DType>& p, const Rotation& r, const Matrix<DType> vs)
{
    if(vs.shape(0) != r.dim())
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    if(vs.shape(0) != p.size())
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    Matrix<DType> ret = r.apply(vs);
    for(size_t i = 0; i < vs.shape(1); i++)
        ret(Col(i)) += p;
    return ret;
}

} // namespace rtc

#endif // __RIGID_BODY_H__
