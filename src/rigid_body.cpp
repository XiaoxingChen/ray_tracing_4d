#include "rigid_body.h"
#include "rotation.h"
#include "axis_aligned_bounding_box.h"
#include "bounding_volume_hierarchy.h"

#include <memory>



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

                if (!ray.valid(t)) return nullptr;
                return std::make_shared<HitRecord>(t, ray(t), ray(t) - center_);
            }
            virtual std::string str() const
            {
                return std::string("c: ") + std::to_string(center_(0)) + ", " + std::to_string(center_(1)) + ", " + std::to_string(center_(2));
            }

            // virtual Vec center() const { return center_; }
            virtual AABB aabb() const
            {
                return AABB(center_ - radius_, center_ + radius_);
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
                auto t_in_out = AxisAlignedBoundingBox::hit(moved_ray, -radius_, radius_);
                bool hit = (t_in_out[0] < t_in_out[1]) && (ray.valid(t_in_out[0]) || ray.valid(t_in_out[1]));
                if(!hit) return nullptr;

                auto p_record = std::make_shared<HitRecord>(ray.origin().size());
                #if 0
                if(ray.valid(t_in_out[0]))
                {
                    std::cout << "outside" << std::endl;
                }
                else
                {
                    std::cout << "inside, t_in:" << t_in_out[0] << std::endl;
                }
                #endif

                FloatType hit_t = ray.valid(t_in_out[0]) ? t_in_out[0] : t_in_out[1];

                // if(! ray.valid(hit_t)) return nullptr;

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
                // std::cout << "n: " << p_record->n.T().str() << "dist: " << min_dist << std::endl;
                p_record->t = hit_t;
                p_record->p = ray(hit_t);
                // p_record->n = norm_vec;
                return p_record;
            }

            // virtual Vec center() const { return center_; }
            virtual AABB aabb() const
            {
                size_t dim(center_.size());
                Mat vertices(radius_.matmul(Vec::ones(1 << dim).T()));
                for(size_t i = 0; i < dim; i++)
                {
                    for(size_t j = 0; j < vertices.shape(1); j++)
                    {
                        if((j & (1 << i)) > 0)
                            vertices(i, j) -= (2 * radius_(i));
                    }
                }
                vertices = orientation_.apply(vertices);
                vertices += center_.matmul(Vec::ones(1 << dim).T());

                AABB box(dim);
                box.extend(vertices);
                return box;
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
        vertices_global_frame_(orientation.apply(vertices)),
        indices_(indices)
        // , aabb_(position.size())
    {
        for(size_t i = 0; i < vertices_global_frame_.shape(1); i++)
        {
            vertices_global_frame_(Col(i)) = vertices_global_frame_(Col(i)) + position_;
        }
        // aabb_.extend(vertices_global_frame_);
    }

    virtual HitRecordPtr hit(const Ray& ray) const
    {
        // if(!aabb_.hit(ray)) return nullptr;

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
                mat_a(Col(i)) = vertices_global_frame_(Col(vertex_indices.at(i)));
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
            norm_complement(Col(i)) =
                vertices_global_frame_(Col(indices_.at(closest_prim_idx).at(i + 1)))
                - vertices_global_frame_(Col(indices_.at(closest_prim_idx).at(i)));
        }
        ret->p = ray(min_t);
        ret->t = min_t;
        ret->n = orthogonalComplement(norm_complement);
        return ret;
    }

    // virtual Vec center() const { return position_; }
    virtual AABB aabb() const
    {
        AABB box(position_.size());
        box.extend(vertices_global_frame_);
        return box;
    }
private:
    Vec position_;
    Rotation orientation_;
    Mat vertices_global_frame_;
    std::vector<std::vector<size_t>> indices_;
    // AxisAlignedBoundingBox aabb_;
};

class PolygonPrimitive: public RigidBody
{
public:
    PolygonPrimitive(std::shared_ptr<Mat> vertex_buffer, std::vector<size_t> indices)
        :p_vertex_buffer_(vertex_buffer), indices_(indices)
        // , norm_(indices.size())
        {   }

    virtual HitRecordPtr hit(const Ray& ray) const
    {
        return hitPrimitivePolygon(ray, p_vertex_buffer_, indices_);
    }

    virtual void multiHit(const Ray& ray, std::vector<HitRecordPtr>& records) const
    {
        records.push_back(hitPrimitivePolygon(ray, p_vertex_buffer_, indices_));
    }

    size_t dim() const { return indices_.size(); }

    virtual AABB aabb() const
    {
        AABB box(dim());
        for(auto & idx: indices_)
        {
            box.extend((*p_vertex_buffer_)(Col(idx)));
        }
        return box;
    }
private:
    std::shared_ptr<Mat> p_vertex_buffer_;
    std::vector<size_t> indices_;
    // UnitVec norm_;
};

//
// A N-Dimensional rigid body, generated by sweep an N-1 Dimensional
// rigid body along N-th axis with length h.
//
// e.g. A cyllinder can be generated by sweep a circle along Z-axis.
class Prism: public RigidBody
{
public:
#if 0
    Prism(const Vec& p, const Rotation& r, FloatType h,
        std::shared_ptr<Mat>& vertex_buffer,
        const Matrix<uint16_t> index_buffer):
        position_(p), orientation_(r),
        half_h_(0.5 * h),
        p_vertex_buffer_(vertex_buffer),
        index_buffer_(index_buffer)
    {
        // todo, nullptr
        std::shared_ptr<bvh::Node>(new bvh::Node(dim(), nullptr, {0, buffer->size()}));
    }
#endif
    size_t dim() const { return position_.size(); }

    virtual HitRecordPtr hit(const Ray& ray) const
    {
        Ray local_ray(ray.origin() - position_, orientation_.inv().apply(ray.direction()));
        return nullptr;
    }

    virtual AABB aabb() const
    {
        return tree_root_->boundingBox();
    }

private:
    Vec position_;
    Rotation orientation_;
    FloatType half_h_;
    // todo , store hittable buffer or (vertex + index) buffer?
    std::shared_ptr<bvh::Node> tree_root_;

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
                Rotation::fromPlaneAngle(plane(Col(0)), plane(Col(1)), args.back())
                ); //rotation
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

    RigidBodyPtr RigidBody::createPolygonPrimitive(
        std::shared_ptr<Mat> vertex_buffer,
        const std::vector<size_t>& indices )
    {
        return std::make_shared<PolygonPrimitive>(vertex_buffer, indices);
    }

} // namespace rtc
