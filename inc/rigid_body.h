#if !defined(__RIGID_BODY_H__)
#define __RIGID_BODY_H__
#include "mxm/geometry_ray.h"
#include "mxm/rotation.h"
#include "mxm/rigid_transform.h"

#include <memory>
#include <string>
#include "mxm/geometry_primitive.h"
#include "mxm/spatial_aabb.h"

using namespace mxm;
namespace rtc
{

// class HitRecordPtr;
class AxisAlignedBoundingBox;
using AABB = mxm::AxisAlignedBoundingBox;

template<size_t DIM>
class RigidBody;

template<size_t DIM>
using RigidBodyPtr = std::shared_ptr<RigidBody<DIM>>;

struct RigidBodyHitRecord
{
    RigidBodyHitRecord(float_t t_, const std::vector<FloatType>& p_, const std::vector<FloatType>& n_)
    :t(t_), p(p_), n(n_){}
    RigidBodyHitRecord(float_t t_, const Vec& p_, const Vec& n_)
    :t(t_), p(p_), n(n_){}
    RigidBodyHitRecord(size_t dim=3) :t(0), p(dim), n(dim){}
    float_t t; //hit t
    Vec p; //hit point
    Vec n; //normal vector
    size_t prim_idx;
    Vec prim_coord_hit_p;
};
using RigidBodyHitRecordPtr = std::shared_ptr<RigidBodyHitRecord>;


template<size_t DIM>
class RigidBody
{
    public:
        
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
        
        virtual RigidBodyHitRecordPtr hit(const Ray& ray) const = 0;
        virtual void multiHit(const Ray& ray, std::vector<RigidBodyHitRecordPtr>& records) const
        {
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
        };
        // virtual Vec center() const = 0;
        virtual std::string str() const {return "";};
        virtual AABB aabb() const = 0;
        constexpr size_t dim() const {return DIM;}
        virtual RigidTransform<FloatType,DIM> pose() const {
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
            return RigidTransform<FloatType,DIM>::identity();
        }

        static RigidBodyPtr<DIM> choose(Types type, size_t dimension, const std::vector<FloatType>& args);
        static RigidBodyPtr<DIM> choose(Types type, const Vec& position, const Rotation<FloatType,DIM>& orientation, const std::vector<FloatType>& args);
        static RigidBodyPtr<DIM> createPrimitiveMesh(const Vec& position, const Rotation<FloatType,DIM>& orientation, std::shared_ptr<Mat>& vertices, std::shared_ptr<Matrix<size_t>>& indices);
        static RigidBodyPtr<DIM> createPolygonPrimitive(std::shared_ptr<Mat> vertex_buffer, std::shared_ptr<Matrix<size_t>>& indices, size_t prim_idx=0);
        static RigidBodyPtr<DIM> createPrism(const Vec& p, const Rotation<FloatType,DIM>& r, FloatType h, std::shared_ptr<Mat>& vertex_buffer, std::shared_ptr<Matrix<size_t>>& vertex_index_buffer);
};

template<size_t DIM>
typename RigidBody<DIM>::HitRecordPtr
hitPrimitivePolygon(
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

    auto ret = std::make_shared<RigidBody<DIM>::HitRecord>(
        intersection_t, ray(intersection_t), norm);

    return ret;
}


#if 0
template<typename DType>
Matrix<DType> rigidBodyTransform(const Vector<DType>& p, const Rotation<FloatType,DIM>& r, const Matrix<DType> vs)
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
#endif

} // namespace rtc

#endif // __RIGID_BODY_H__
