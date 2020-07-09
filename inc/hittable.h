#ifndef __HISTTABLE_H__
#define __HISTTABLE_H__
#include "ray.h"
#include <memory>
#include <vector>
#include "material.h"
#include "rigid_body.h"

namespace rtc
{

class Hittable
{
    public:
        Hittable(RigidBodyPtr&& p_rigid, MaterialPtr&& p_material):
            rigid_body_(p_rigid),
            material_(p_material){}

        Hittable(Hittable&& other):
            rigid_body_(other.rigid_body_),
            material_(other.material_){}

        Hittable(const Hittable& other):
            rigid_body_(other.rigid_body_),
            material_(other.material_){}

        // Hittable():
        //     rigid_body_(RigidBody::choose(RigidBody::SPHERE)),
        //     material_(Material::choose(Material::METAL)){}
        
        const RigidBody& rigidBody() const { return *rigid_body_; }
        const Material& material() const { return *material_; }

        struct HitRecord {
            HitRecord(Vector3 attenuation_, const Ray& scattered_)
            :attenuation(attenuation_), scattered(scattered_){}
            Vector3 attenuation;
            Ray scattered;
        };
        using HitRecordPtr = std::shared_ptr<HitRecord>;

    protected:
        RigidBodyPtr rigid_body_;
        MaterialPtr material_;
        
};

// using HittablePtr = std::shared_ptr<Hittable>;

class HitManager
{
    public:

        Hittable::HitRecordPtr hit(const Ray& ray) const
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
                nearest_obj->material().scatter(ray, nearest_hit->p, nearest_hit->n));
            return ret;
        }
        
        void addHittables(RigidBodyPtr&& p_rigid, MaterialPtr&& p_material)
        {
            hittables_.push_back(Hittable(std::forward<RigidBodyPtr>(p_rigid), std::forward<MaterialPtr>(p_material)));
        }

    private:
        // std::vector<HittablePtr> hittables_;
        std::vector<Hittable> hittables_;
};

} //namespace rtc


#endif