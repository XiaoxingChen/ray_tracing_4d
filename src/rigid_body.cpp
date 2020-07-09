#include "rigid_body.h"

namespace rtc
{
    class Sphere: public RigidBody
    {
        public:
            Sphere(
                const Vector3& center=V3{0,0,0}, 
                float_t radius=3)
                :center_(center), radius_(radius){}

            virtual HitRecordPtr hit(const Ray& ray) const
            {
                auto oc = center_ - ray.origin();
                float dist = oc.cross(ray.direction()).norm();
                
                if(dist >= radius_) return nullptr;
                float_t t = oc.dot(ray.direction());
                if(t < 0) return nullptr;
                if(oc.norm() > radius_ + 1e-4)
                    t -= sqrt(radius_ * radius_ - dist * dist);
                else
                    t += sqrt(radius_ * radius_ - dist * dist);
                
                if (!ray.valid(t)) 
                {
                    std::cout << "invalid t: " << t 
                    << ", sqrt: " << sqrt(radius_ * radius_ - dist * dist)
                    << ", center_: " << center_
                    << ", r: " << radius_
                    << ", dot(): " << oc.dot(ray.direction())
                    << ", ray.origin(): " << ray.origin()
                    << ", ray.direction(): " << ray.direction()
                    << std::endl;
                    return nullptr;
                }
                return std::make_shared<HitRecord>(t, ray(t), ray(t) - center_);
            }
            virtual std::string str() const
            {
                return std::string("c: ") + std::to_string(center_.x()) + ", " + std::to_string(center_.y()) + ", " + std::to_string(center_.z());
            }
        private:
            Vector3 center_;
            float radius_;
    };    

    RigidBodyPtr RigidBody::choose(Types type, const Vector3& center, const Vector3& size)
    {
        if(type == RigidBody::SPHERE)
            return std::make_shared<Sphere>(center, size.at(0));
        
        return std::make_shared<Sphere>();
    }
} // namespace rtc
