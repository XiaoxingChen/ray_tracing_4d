#ifndef __HISTTABLE_H__
#define __HISTTABLE_H__
#include "ray.h"
#include <memory>
#include <vector>
#include "material.h"
#include "rigid_body.h"
// #include "bounding_volume_hierarchy.h"


namespace rtc
{
namespace bvh
{
    class Node;
} // namespace bvh


class Hittable
{
    public:
        Hittable(RigidBodyPtr p_rigid, MaterialPtr p_material, const std::string& id=""):
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

        const RigidBody& rigidBody() const { return *rigid_body_; }
        const Material& material() const { return *material_; }

        struct HitRecord {
            HitRecord(const Pixel& attenuation_, const Ray& scattered_, FloatType hit_t_)
            :attenuation(attenuation_), scattered(scattered_), hit_t(hit_t_) {}
            Pixel attenuation;
            Ray scattered;
            FloatType hit_t;
        };
        using HitRecordPtr = std::shared_ptr<HitRecord>;

    protected:
        RigidBodyPtr rigid_body_;
        MaterialPtr material_;
        std::string id_;

};

// using HittablePtr = std::shared_ptr<Hittable>;

class HitManager
{
    public:

        // virtual Hittable::HitRecordPtr hit(Ray& ray) const {};
        virtual Hittable::HitRecordPtr hit(Ray& ray) const
        {
            RigidBody::HitRecordPtr nearest_hit = nullptr;
            const Hittable* nearest_obj = nullptr;
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
            auto ret = std::make_shared<Hittable::HitRecord>(
                nearest_obj->material().attenuation(),
                nearest_obj->material().scatter(ray, nearest_hit->p, nearest_hit->n),
                nearest_hit->t);
            return ret;
        }

        void addHittables(const RigidBodyPtr& p_rigid, const MaterialPtr& p_material)
        {
            hittables_.push_back(Hittable(p_rigid, p_material));
        }

    private:
        // std::vector<HittablePtr> hittables_;
        std::vector<Hittable> hittables_;
};

using HittableBuffer = std::vector<Hittable>;
using HittableBufferPtr = std::shared_ptr<HittableBuffer>;
using HitManagerPtr = HitManager*;

} //namespace rtc


#endif