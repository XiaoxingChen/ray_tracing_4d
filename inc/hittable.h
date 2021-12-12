#ifndef __HISTTABLE_H__
#define __HISTTABLE_H__
#include "mxm/geometry_ray.h"
#include <memory>
#include <vector>
#include "material.h"
#include "rigid_body.h"
// #include "bounding_volume_hierarchy.h"
using namespace mxm;

namespace rtc
{
namespace bvh
{
template<size_t DIM>
class Node;
} // namespace bvh


template<size_t DIM>
class Hittable
{
    public:
        Hittable(RigidBodyPtr<DIM> p_rigid, MaterialPtr p_material, const std::string& id=""):
            rigid_body_(p_rigid),
            material_(p_material),
            id_(id){}

        Hittable(Hittable&& other):
            rigid_body_(other.rigid_body_),
            material_(other.material_),
            id_(other.id_){}

        Hittable(const Hittable& other):
            rigid_body_(other.rigid_body_),
            material_(other.material_),
            id_(other.id_){}

        // Hittable():
        //     rigid_body_(RigidBody::choose(RigidBody::SPHERE)),
        //     material_(Material::choose(Material::METAL)){}
        void operator = (const Hittable& rhs)
        {
            rigid_body_ = rhs.rigid_body_;
            material_ = rhs.material_;
            id_ = rhs.id_;
        }

        const RigidBody<DIM>& rigidBody() const { return *rigid_body_; }
        const Material& material() const { return *material_; }

    protected:
        RigidBodyPtr<DIM> rigid_body_;
        MaterialPtr material_;
        std::string id_;

};

struct HittableHitRecord {
    HittableHitRecord(const Pixel& attenuation_, const Ray& scattered_, FloatType hit_t_)
    :attenuation(attenuation_), scattered(scattered_), hit_t(hit_t_) {}
    Pixel attenuation;
    Ray scattered;
    FloatType hit_t;
};
using HittableHitRecordPtr = std::shared_ptr<HittableHitRecord>;

// using HittablePtr = std::shared_ptr<Hittable>;

template<size_t DIM>
class HitManager
{
    public:

        // virtual HittableHitRecordPtr hit(Ray& ray) const {};
        virtual HittableHitRecordPtr hit(Ray& ray) const
        {
            RigidBodyHitRecordPtr nearest_hit = nullptr;
            const Hittable<DIM>* nearest_obj = nullptr;
            for(const auto & obj: hittables_)
            {
                auto record = obj.rigidBody().hit(ray);
                if (record == nullptr) continue;
                if((nullptr == nearest_hit && nullptr == nearest_obj)
                    || record->t < nearest_hit->t)
                {
                    nearest_hit = record;
                    nearest_obj = &obj;
                }

            }
            if(nullptr == nearest_obj)
            {
                return nullptr;
            }
            auto ret = std::make_shared<HittableHitRecord>(
                nearest_obj->material().attenuation(*nearest_hit),
                nearest_obj->material().scatter(ray, *nearest_hit),
                nearest_hit->t);
            return ret;
        }

        void addHittables(const RigidBodyPtr<DIM>& p_rigid, const MaterialPtr& p_material)
        {
            hittables_.push_back(Hittable<DIM>(p_rigid, p_material));
        }

    private:
        // std::vector<HittablePtr> hittables_;
        std::vector<Hittable<DIM>> hittables_;
};


template<size_t DIM> using HittableBuffer = std::vector<Hittable<DIM>>;
template<size_t DIM> using HittableBufferPtr = std::shared_ptr<HittableBuffer<DIM>>;
template<size_t DIM> using HitManagerPtr = HitManager<DIM>*;

} //namespace rtc


#endif