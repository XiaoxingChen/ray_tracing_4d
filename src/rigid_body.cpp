#include "rigid_body.h"

namespace rtc
{
    class Sphere: public RigidBody
    {
        public:
            Sphere(
                const std::vector<FloatType>& center,
                FloatType radius=3)
                :center_(center), radius_(radius){}

            Sphere(size_t dimension=3)
                :center_({0,0,0}), radius_(2){}

            virtual HitRecordPtr hit(const Ray& ray) const
            {
                Vec oc = center_ - ray.origin();

                Vec closest_pt(ray.origin() + ray.direction() * oc.dot(ray.direction()));
                FloatType dist = (closest_pt - center_).norm(2);

                if(dist >= radius_) return nullptr;
                FloatType t = oc.dot(ray.direction());
                if(t < 0) return nullptr;
                if(oc.norm() > radius_ + eps())
                    t -= sqrt(radius_ * radius_ - dist * dist);
                else
                    t += sqrt(radius_ * radius_ - dist * dist);

                if (!ray.valid(t))
                {
                    // std::cout << "invalid t: " << t
                    // << ", sqrt: " << sqrt(radius_ * radius_ - dist * dist)
                    // << ", center_: " << center_
                    // << ", r: " << radius_
                    // << ", dot(): " << oc.dot(ray.direction())
                    // << ", ray.origin(): " << ray.origin()
                    // << ", ray.direction(): " << ray.direction()
                    // << std::endl;
                    return nullptr;
                }
                return std::make_shared<HitRecord>(t, ray(t), ray(t) - center_);
            }
            virtual std::string str() const
            {
                return std::string("c: ") + std::to_string(center_(0)) + ", " + std::to_string(center_(1)) + ", " + std::to_string(center_(2));
            }
        private:
            Vec center_;
            FloatType radius_;
    };

    RigidBodyPtr RigidBody::choose(Types type, size_t dimension, const std::vector<FloatType>& args)
    {
        if(type == RigidBody::SPHERE)
            return std::make_shared<Sphere>(std::vector<FloatType>(args.begin(), args.begin() + dimension), args.at(dimension));

        return std::make_shared<Sphere>();
    }
} // namespace rtc
