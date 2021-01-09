#include "rigid_body.h"
#include "rotation.h"
#include "axis_aligned_bounding_box.h"
#include "primitive_geometry.h"



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
                bool hit = (t_min_max[0] < t_min_max[1]) && (0 < t_min_max[0]);
                if(!hit) return nullptr;

                auto p_record = std::make_shared<HitRecord>();
                FloatType hit_t = t_min_max[0];

                FloatType min_dist = radius_(0);
                size_t min_axis = 0;
                Vec dist0(radius_ - moved_ray(hit_t));
                Vec dist1(-radius_ - moved_ray(hit_t));
                for(size_t a = 0; a < center_.size(); a++)
                {
                    if(std::min(fabs(dist0(a)), fabs(dist1(a))) > min_dist) continue;
                    min_axis = a;
                    min_dist = std::min(fabs(dist0(a)), fabs(dist1(a)));
                }
                Vec norm_vec(Vec::zeros(center_.size()));
                norm_vec(min_axis) = -moved_ray.direction()(min_axis);

                p_record->n = static_cast<Vec>(orientation_.apply(norm_vec));
                p_record->t = t_min_max[0];
                p_record->p = ray(t_min_max[0]);
                // p_record->n = norm_vec;
                return p_record;
            }

        private:
            Vec center_;
            Vec radius_;
            Rotation orientation_;
    };

class PrimitiveMesh: public RigidBody
{
public:
    PrimitiveMesh(
        const Vec& position,
        const Rotation& orientation,
        const Mat& vertices,
        const std::vector<std::vector<size_t>>& indices)
        :position_(position), orientation_(orientation),
        vertices_global_frame_(orientation.apply(vertices)), indices_(indices)
    {
        for(size_t i = 0; i < vertices_global_frame_.shape(1); i++)
        {
            vertices_global_frame_.set(Col(i), vertices_global_frame_(Col(i)) + position_);
        }
    }

    virtual HitRecordPtr hit(const Ray& ray) const
    {
        FloatType min_t = ray.tMax();
        int closest_prim_idx = -1;
        for(size_t prim_idx = 0; prim_idx < indices_.size(); prim_idx++)
        {
            auto & vertex_indices(indices_.at(prim_idx));
            if(position_.size() != vertex_indices.size())
                throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
            Mat mat_a({position_.size(), position_.size()});
            for(size_t i = 0; i < position_.size(); i++)
            {
                mat_a.set(Col(i), vertices_global_frame_(Col(vertex_indices.at(i))));
            }
            Vec result = intersectEquation(mat_a, ray);
            if(!validIntersect(result)) continue;

            FloatType intersection_t = result(0);

            if(intersection_t < ray.tMin()) continue;
            if (intersection_t >= min_t) continue;

            min_t = intersection_t;
            closest_prim_idx = prim_idx;
        }
        if(closest_prim_idx < 0)
            return nullptr;

        auto ret = std::make_shared<HitRecord>(position_.size());
        Mat norm_complement({position_.size(), position_.size() - 1});
        for(size_t i = 0; i < position_.size() - 1; i++)
        {
            norm_complement.set(Col(i),
                vertices_global_frame_(Col(indices_.at(closest_prim_idx).at(i + 1)))
                - vertices_global_frame_(Col(indices_.at(closest_prim_idx).at(i))));
        }
        ret->p = ray(min_t);
        ret->t = min_t;
        ret->n = orthogonalComplement(norm_complement);
        return ret;
    }
private:
    Vec position_;
    Rotation orientation_;
    Mat vertices_global_frame_;
    std::vector<std::vector<size_t>> indices_;
};

#if 1
    RigidBodyPtr RigidBody::choose(Types type, size_t dimension, const std::vector<FloatType>& args)
    {
        if(type == RigidBody::SPHERE)
            return std::make_shared<Sphere>(std::vector<FloatType>(args.begin(), args.begin() + dimension), args.at(dimension));

        if(type == RigidBody::RECTANGLE)
        {
            Mat plane({dimension,2}, std::vector<FloatType>(args.begin() + 2*dimension, args.begin() + 4*dimension));
            return std::make_shared<Rectangle>(
                std::vector<FloatType>(args.begin(), args.begin() + dimension), //center
                std::vector<FloatType>(args.begin() + dimension, args.begin() + 2*dimension), //radius
                Rotation::fromPlaneAngle(plane.block({},{0,1}), plane.block({},{1,2}), args.back())); //rotation
        }


        return std::make_shared<Sphere>();
    }
    #endif

    RigidBodyPtr RigidBody::choose(Types type, VecIn position, const Rotation& orientation, const std::vector<FloatType>& args)
    {
        if(position.size() != orientation.dim())
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

        if(type == RigidBody::SPHERE)
            return std::make_shared<Sphere>(position, args.at(0));

        if(type == RigidBody::RECTANGLE)
        {
            return std::make_shared<Rectangle>(
                position, //center
                args, //radius
                orientation); //rotation
        }


        return std::make_shared<Sphere>();
    }

    RigidBodyPtr RigidBody::createPrimitiveMesh(VecIn position, const Rotation& orientation, const Mat& vertices, const std::vector<std::vector<size_t>>& indices)
    {
        return std::make_shared<PrimitiveMesh>(position, orientation, vertices, indices);
    }

} // namespace rtc
