#include "rigid_body.h"
#include "rotation.h"
#include "axis_aligned_bounding_box.h"


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

    class Rectangle: public RigidBody
    {
        public:
            Rectangle(
                const std::vector<FloatType>& center,
                const std::vector<FloatType>& radius,
                const Rotation& orientation)
                :center_(center), radius_(radius), orientation_(orientation){}

            Rectangle(size_t dim=3)
                :center_(Vec::zeros(dim)), radius_(Vec::ones(dim)*.5), orientation_(Rotation::Identity(dim)){}

            virtual HitRecordPtr hit(const Ray& ray) const
            {
                Ray moved_ray(
                    orientation_.inv().apply(ray.origin() - center_),
                    orientation_.inv().apply(ray.direction()));
                auto t_min_max = AxisAlignedBoundingBox::hit(moved_ray, -radius_, radius_);
                bool hit = t_min_max[0] < t_min_max[1];
                if(!hit) return nullptr;

                auto p_record = std::make_shared<HitRecord>();
                p_record->t = t_min_max[0];
                p_record->p = ray(t_min_max[0]);

                FloatType min_dist = center_(0);
                size_t min_axis = 0;
                Vec dist0(center_ + radius_ - p_record->p);
                Vec dist1(center_ - radius_ - p_record->p);
                for(size_t a = 0; a < center_.size(); a++)
                {
                    if(std::min(fabs(dist0(a)), fabs(dist1(a))) > min_dist) continue;
                    min_axis = a;
                    min_dist = std::min(fabs(dist0(a)), fabs(dist1(a)));
                }
                Vec norm_vec(Vec::zeros(center_.size()));
                norm_vec(min_axis) = -ray.direction()(min_axis);
                // p_record->n = static_cast<Vec>(orientation_.apply(norm_vec));
                p_record->n = norm_vec;
                return p_record;
            }

        private:
            Vec center_;
            Vec radius_;
            Rotation orientation_;
    };

    RigidBodyPtr RigidBody::choose(Types type, size_t dimension, const std::vector<FloatType>& args)
    {
        if(type == RigidBody::SPHERE)
            return std::make_shared<Sphere>(std::vector<FloatType>(args.begin(), args.begin() + dimension), args.at(dimension));

        if(type == RigidBody::RECTANGLE)
            return std::make_shared<Rectangle>(
                std::vector<FloatType>(args.begin(), args.begin() + dimension), //center
                std::vector<FloatType>(args.begin() + dimension, args.begin() + 2*dimension), //radius
                Rotation(Mat({dimension, 2},
                    std::vector<FloatType>(args.begin() + 2*dimension, args.begin() + 4*dimension)), //rotation plane
                    args.back())); //rotation angle

        return std::make_shared<Sphere>();
    }
} // namespace rtc
